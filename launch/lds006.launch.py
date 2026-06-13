from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lds006_python_driver',
            executable='lds006_node',         # Phải trùng khớp chính xác với khai báo ở setup.py
            name='lds006_driver_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},      
                {'frame_id': 'laser_frame'}    
            ]
        )
    ])