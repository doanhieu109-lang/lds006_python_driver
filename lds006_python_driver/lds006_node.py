#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import threading

class LDS006Ros2Node(Node):
    def __init__(self):
        super().__init__('lds006_node')
        
        # Khai báo các Tham số cấu hình (Parameters)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser_frame')
        
        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Khởi tạo Publisher để đẩy dữ liệu lên RViz2
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Bê nguyên toàn bộ các biến khởi tạo từ form code cũ của bạn
        self.count = 0
        self.arraysize = 100
        self.values = [0] * self.arraysize
        
        # Trong ROS 2, các điểm không hợp lệ hoặc không có vật cản được biểu diễn bằng vô cực float('inf')
        self.distances = [float('inf')] * 360
        self.intensities = [0.0] * 360
        self.min_reflectivity = 10

        # Kết nối cổng Serial với đúng tham số của bạn (baudrate=115200, timeout=5)
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=5)
            # Gửi lệnh kích hoạt Lidar giống hệt code cũ
            self.ser.write(b'$')
            self.ser.write(b"startlds$")
            self.get_logger().info(f"Đã kết nối và kích hoạt LDS-006 tại cổng: {self.port}")
        except Exception as e:
            self.get_logger().error(f"Không thể mở cổng Serial: {e}")
            return

        # Tạo một luồng riêng (Thread) để chạy vòng lặp đọc byte liên tục không làm treo ROS
        self.running = True
        self.serial_thread = threading.Thread(target=self.serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def get_int(self, lb, hb):
        return lb | (hb << 8)

    def process_lidar_data(self):
        # Giữ nguyên thuật toán trích xuất dữ liệu từ mảng values của bạn
        angle = (self.values[1] - 0xA0) * 4
        speed = self.get_int(self.values[2], self.values[3])
        
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

        checksum2 = 0
        for x in range(20):
            checksum2 = checksum2 + self.values[x]
      
        checksum1 = self.get_int(self.values[20], self.values[21])
        
        if checksum1 == checksum2 and angle < 360:
            # Khi Lidar quét quay về góc 0, xuất bản tin vòng quét cũ lên RViz2
            if angle == 0:
                self.publish_scan()
                self.distances = [float('inf')] * 360
                self.intensities = [0.0] * 360
                
            for x in range(4):
                if angle + x < 360:
                    # Giữ nguyên bộ lọc điều kiện cường độ phản xạ (> 10) của bạn
                    if reflectivity[x] > self.min_reflectivity and distance[x] > 0:
                        # Đổi đơn vị từ mm sang mét để RViz2 hiểu đúng khoảng cách hình học
                        self.distances[angle+x] = float(distance[x]) / 1000.0
                        self.intensities[angle+x] = float(reflectivity[x])
                    else:
                        self.distances[angle+x] = float('inf')
                        self.intensities[angle+x] = 0.0

    def publish_scan(self):
        # Đóng gói dữ liệu vào cấu trúc chuẩn LaserScan của ROS 2
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi * (359.0 / 360.0)
        msg.angle_increment = (2.0 * math.pi) / 360.0
        msg.time_increment = 0.0
        msg.scan_time = 0.2
        msg.range_min = 0.1  # 10 cm
        msg.range_max = 6.0  # 6 m
        
        msg.ranges = self.distances
        msg.intensities = self.intensities
        
        self.publisher_.publish(msg)

    def serial_loop(self):
        # Giữ nguyên hoàn toàn logic đọc từng byte b = ser.read() từ code cũ của bạn
        while rclpy.ok() and self.running:
            try:
                b = self.ser.read(1)
                if not b:
                    continue
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
                self.count = self.count + 1
            except Exception as e:
                pass

    def stop(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                # Gửi lệnh tắt Lidar an toàn trước khi thoát chương trình
                self.ser.write(b"stoplds$")
                self.ser.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = LDS006Ros2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()