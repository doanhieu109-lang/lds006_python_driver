import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lds006_python_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # THÊM DÒNG NÀY ĐỂ ROS 2 NHẬN DIỆN THƯ MỤC LAUNCH:
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anvietdektop',
    maintainer_email='doanhieu109@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tên executable ở đây đang là 'lds006_node'
            'lds006_node = lds006_python_driver.lds006_node:main'
        ],
    },
)