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
        
        # Thay thế đoạn khai báo thông số cũ bằng cấu hình nhận Parameter của ROS 2
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'laser_frame')

        # Đọc giá trị cấu hình (nếu terminal truyền vào cổng nào thì hệ thống sẽ lấy cổng đó)
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.min_reflectivity = 10
        
        # Khởi tạo mảng lưu trữ 360 độ (mặc định là vô cực nếu chưa có dữ liệu)
        self.distances = [float('inf')] * 360
        self.intensities = [0.0] * 360
        
        # Bộ đệm đọc Serial giống như script gốc của cháu
        self.count = 0
        self.arraysize = 100
        self.values = [0] * self.arraysize
        
        # Kết nối phần cứng và khởi động cảm biến
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=5)
            self.ser.write(b'$')
            self.ser.write(b"startlds$")
            self.get_logger().info(f"Đã kết nối LDS-006 tại cổng {self.port} và gửi lệnh khởi động.")
        except Exception as e:
            self.get_logger().error(f"Không thể kết nối cổng Serial: {e}")
            return

        # Tạo một luồng riêng (Thread) để đọc dữ liệu Serial liên tục không chặn ROS
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_loop)
        self.serial_thread.start()

    def get_int(self, lb, hb):
        return lb | (hb << 8)

    def process_lidar_data(self):
        angle = (self.values[1] - 0xA0) * 4
        # speed = self.get_int(self.values[2], self.values[3]) # Có thể dùng nếu cần theo dõi vận tốc vòng quay
        
        distance = [0] * 4
        reflectivity = [0] * 4
        
        distance[0]     = self.get_int(self.values[4], self.values[5])
        reflectivity[0] = self.get_int(self.values[6], self.values[7])

        distance[1]     = self.get_int(self.values[8], self.values[9])
        reflectivity[1] = self.get_int(self.values[10], self.values[11])

        distance[2]     = self.get_int(self.values[12], self.values[13])
        reflectivity[2] = self.get_int(self.values[14], self.values[15])

        distance[3]     = self.get_int(self.values[16], self.values[17])
        reflectivity[3] = self.get_int(self.values[18], self.values[19])

        # Tính toán kiểm tra Checksum
        checksum2 = 0
        for x in range(20):
            checksum2 = checksum2 + self.values[x]
      
        checksum1 = self.get_int(self.values[20], self.values[21])
        
        if checksum1 == checksum2 and angle < 360:
            # Khi Lidar quay về góc 0, tiến hành xuất bản (publish) vòng quét cũ ra ROS 2
            if angle == 0:
                self.publish_scan()
                # Reset lại bộ đệm cho vòng quét mới
                self.distances = [float('inf')] * 360
                self.intensities = [0.0] * 360
                
            # Đổ dữ liệu 4 điểm quét mới vào mảng
            for x in range(4):
                if angle + x < 360:
                    if reflectivity[x] > self.min_reflectivity and distance[x] > 0:
                        # Đổi từ mm sang mét theo chuẩn ROS 2
                        self.distances[angle + x] = float(distance[x]) / 1000.0
                        self.intensities[angle + x] = float(reflectivity[x])
                    else:
                        self.distances[angle + x] = float('inf')
        else:
            self.get_logger().warn(f"Dữ liệu không hợp lệ hoặc sai checksum: góc {angle}")

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # Cấu hình thông số hình học cho Lidar quay 360 độ
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi * (359.0 / 360.0)
        scan_msg.angle_increment = (2.0 * math.pi) / 360.0
        
        # Các thông số vật lý của tia quét (có thể điều chỉnh tùy thực tế cảm biến)
        scan_msg.scan_time = 0.2        # Giả định tốc độ quét 5Hz (5 vòng / giây)
        scan_msg.range_min = 0.1        # Khoảng cách đo tối thiểu (mét)
        scan_msg.range_max = 6.0        # Khoảng cách đo tối đa (mét)
        
        # Gán mảng dữ liệu đã xử lý
        scan_msg.ranges = self.distances
        scan_msg.intensities = self.intensities
        
        # Thực hiện phát dữ liệu lên hệ thống ROS 2
        self.publisher_.publish(scan_msg)

    def serial_loop(self):
        """Vòng lặp chạy ngầm để liên tục đọc và phân tích dữ liệu luồng từ cổng Serial"""
        while rclpy.ok() and self.running:
            if self.ser.in_waiting > 0:
                b = self.ser.read()
                val = int.from_bytes(b, byteorder='big')
                
                if val == 0xFA and self.count > 21:
                    if self.count == 22:
                        self.process_lidar_data()
                    self.count = 0
                    self.values = [0] * self.arraysize
                    self.values[0] = val
                else:
                    if self.count < self.arraysize:
                        self.values[self.count] = val
                self.count += 1

    def stop(self):
        # Dừng luồng đọc và đóng cổng kết nối an toàn khi tắt Node
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info("Đã đóng kết nối an toàn với thiết bị.")

def main(args=None):
    rclpy.init(args=args)
    node = LDS006DriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node LDS006 đang dừng bằng KeyboardInterrupt...')
    finally:
        # Kiểm tra xem hệ thống đã shutdown chưa trước khi gọi để tránh lỗi rcl_shutdown
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()