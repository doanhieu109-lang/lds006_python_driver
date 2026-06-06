#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import threading

class LDS006DriverNode(Node):
    def __init__(self):
        super().__init__('lds006_driver_node')
        
        # Khai báo Publisher cho dữ liệu LaserScan
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Cấu hình nhận Parameter của ROS 2 từ Terminal
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.min_reflectivity = 10
        
        # Khởi tạo mảng lưu trữ 360 độ (mặc định là vô cực theo chuẩn ROS 2)
        self.distances = [float('inf')] * 360
        self.intensities = [0.0] * 360
        
        # Kết nối phần cứng cổng Serial
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
            self.get_logger().info(f"Đã kết nối thành công LDS-006 tại cổng {self.port}")
        except Exception as e:
            self.get_logger().error(f"Không thể kết nối cổng Serial: {e}")
            return

        # Tạo một luồng riêng (Thread) để đọc dữ liệu Serial liên tục không chặn ROS 2
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def get_int(self, lb, hb):
        return lb | (hb << 8)

    def process_lidar_packet(self, packet_values):
        """Hàm bóc tách gói tin 22-bytes chuẩn chỉnh"""
        angle = (packet_values[1] - 0xA0) * 4
        
        # Lọc bỏ ngay các gói tin có góc tính toán sai lệch cấu trúc
        if angle < 0 or angle >= 360:
            return

        distance = [0] * 4
        reflectivity = [0] * 4
        
        distance[0]     = self.get_int(packet_values[4], packet_values[5])
        reflectivity[0] = self.get_int(packet_values[6], packet_values[7])

        distance[1]     = self.get_int(packet_values[8], packet_values[9])
        reflectivity[1] = self.get_int(packet_values[10], packet_values[11])

        distance[2]     = self.get_int(packet_values[12], packet_values[13])
        reflectivity[2] = self.get_int(packet_values[14], packet_values[15])

        distance[3]     = self.get_int(packet_values[16], packet_values[17])
        reflectivity[3] = self.get_int(packet_values[18], packet_values[19])

        # Tính toán kiểm tra Checksum (Tổng 20 byte đầu)
        checksum2 = sum(packet_values[:20])
        checksum1 = self.get_int(packet_values[20], packet_values[21])
        
        if checksum1 == checksum2:
            # Khi Lidar quay về góc 0, xuất bản (publish) dữ liệu vòng quét cũ lên RViz 2
            if angle == 0:
                self.publish_scan()
                # Reset mảng cho vòng quét tiếp theo
                self.distances = [float('inf')] * 360
                self.intensities = [0.0] * 360
                
            # Đổ dữ liệu 4 điểm quét chi tiết vào mảng
            for x in range(4):
                current_angle = angle + x
                if current_angle < 360:
                    if reflectivity[x] > self.min_reflectivity and 100 <= distance[x] <= 6000:
                        # Đổi từ mm sang mét theo chuẩn hệ mét của ROS 2 LaserScan
                        self.distances[current_angle] = float(distance[x]) / 1000.0
                        self.intensities[current_angle] = float(reflectivity[x])
                    else:
                        self.distances[current_angle] = float('inf')
                        self.intensities[current_angle] = 0.0
        else:
            # Tần suất cảnh báo lỗi được giới hạn (throttle) 2 giây/lần để không làm nghẽn terminal
            self.get_logger().warn(f"Phát hiện gói dữ liệu sai checksum tại góc: {angle}", throttle_duration_sec=2.0)

    def publish_scan(self):
        """Hàm đóng gói thông điệp LaserScan lên hệ thống mạng ROS 2"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Cấu hình thông số hình học cho Lidar quay tròn 360 độ (Radian)
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi * (359.0 / 360.0)
        scan_msg.angle_increment = (2.0 * math.pi) / 360.0
        
        # Các thông số vật lý của tia quét
        scan_msg.scan_time = 0.2        
        scan_msg.range_min = 0.1        # 10 cm
        scan_msg.range_max = 6.0        # 6 mét
        
        # Gán mảng dữ liệu đã xử lý
        scan_msg.ranges = self.distances
        scan_msg.intensities = self.intensities
        
        # Thực hiện phát dữ liệu công khai
        self.publisher_.publish(scan_msg)

    def serial_loop(self):
        """Vòng lặp đọc luồng dữ liệu tối ưu: Quét tìm 0xFA -> Đọc luôn khối 21-bytes"""
        # Gửi lệnh kích hoạt Lidar phần cứng bắt đầu phát
        self.ser.write(b'$')
        self.ser.write(b"startlds$")
        
        # Giải phóng/Xóa sạch toàn bộ bộ đệm dồn ứ cũ trong phần cứng con Pi
        self.ser.reset_input_buffer()
        self.get_logger().info("Đã gửi lệnh kích hoạt mô-tơ Lidar thành công. Đang xử lý luồng dữ liệu...")

        while rclpy.ok() and self.running:
            try:
                # 1. Quét tìm duy nhất 1 byte mở đầu có giá trị 0xFA
                b = self.ser.read(1)
                if not b or b[0] != 0xFA:
                    continue
                
                # 2. Đọc luôn một khối 21-bytes còn lại xếp hàng phía sau
                packet = self.ser.read(21)
                if len(packet) < 21:
                    continue
                
                # 3. Tạo mảng 22-bytes hoàn chỉnh và đẩy đi giải mã
                packet_values = [0xFA] + list(packet)
                self.process_lidar_packet(packet_values)

            except Exception as e:
                self.get_logger().error(f"Lỗi xảy ra trong luồng đọc Serial: {e}")

    def stop(self):
        """Hàm dừng an toàn: Tắt động cơ Lidar và ngắt kết nối cổng"""
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                # Gửi lệnh lịch sự bảo Lidar dừng quay trước khi rút nguồn
                self.ser.write(b"stoplds$")
            except:
                pass
            self.ser.close()
        self.get_logger().info("Đã đóng kết nối an toàn với thiết bị.")

def main(args=None):
    rclpy.init(args=args)
    node = LDS006DriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hệ thống nhận lệnh ngắt Ctrl+C. Đang dừng Node...')
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()