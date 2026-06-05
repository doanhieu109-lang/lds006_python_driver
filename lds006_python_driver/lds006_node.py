#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import threading
import math

class LDS006DriverNode(Node):
    def __init__(self):
        super().__init__('lds006_node')
        
        # 1. Khai báo và lấy các tham số (Parameters) từ Terminal
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # 2. Khởi tạo cấu hình mảng dữ liệu Lidar
        self.min_reflectivity = 10
        self.distances = [float('inf')] * 360  # ROS 2 sử dụng inf cho các điểm không có vật cản

        # 3. Tạo Publisher để đẩy dữ liệu lên Topic /scan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # 4. Kết nối tới cổng Serial của Lidar
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
            self.get_logger().info(f"Kết nối thành công cổng Serial: {self.port} ở tốc độ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Không thể kết nối cổng Serial: {e}")
            raise e

        # 5. Tạo một luồng (Thread) chạy ngầm để đọc Serial độc lập, tránh làm nghẽn ROS 2
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def read_serial_loop(self):
        """Vòng lặp đọc dữ liệu Lidar tối ưu bằng khối 22-bytes"""
        # Gửi lệnh kích hoạt Lidar phát dữ liệu
        self.ser.write(b'$')
        self.ser.write(b"startlds$")
        
        # Xóa sạch bộ đệm dồn ứ lúc cắm/rút dây trước khi vào vòng lặp
        self.ser.reset_input_buffer()
        self.get_logger().info("Mô-tơ Lidar kích hoạt thành công. Đang thu thập dữ liệu...")

        while self.running and rclpy.ok():
            try:
                # Tìm byte bắt đầu gói tin 0xFA
                b = self.ser.read(1)
                if not b or b[0] != 0xFA:
                    continue

                # Khi đã thấy 0xFA, đọc nốt 21 byte còn lại để đủ khung dữ liệu 22-bytes
                packet = self.ser.read(21)
                if len(packet) < 21:
                    continue

                # Tạo mảng dữ liệu hoàn chỉnh giống cấu trúc code cũ của bạn
                values = [0xFA] + list(packet)

                # Tính toán góc cơ sở (Mỗi gói tin chứa thông tin của 4 điểm liên tiếp)
                angle = (values[1] - 0xA0) * 4
                if angle < 0 or angle >= 360:
                    continue

                # Kiểm tra mã Checksum
                checksum2 = sum(values[:20])
                checksum1 = values[20] | (values[21] << 8)

                if checksum1 == checksum2:
                    # Nếu Lidar quay về góc 0 (Hoàn thành 1 vòng quét cũ), phát dữ liệu lên RViz
                    if angle == 0:
                        self.publish_scan()
                        self.distances = [float('inf')] * 360  # Reset mảng cho vòng quét mới

                    # Trích xuất dữ liệu của 4 điểm đo trong gói tin
                    for x in range(4):
                        offset = 4 + (x * 4)
                        distance_mm = values[offset] | (values[offset + 1] << 8)
                        reflectivity = values[offset + 2] | (values[offset + 3] << 8)

                        # Lọc nhiễu khoảng cách ảo và kiểm tra độ phản xạ
                        if reflectivity > self.min_reflectivity and 100