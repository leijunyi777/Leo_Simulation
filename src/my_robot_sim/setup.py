from setuptools import find_packages, setup
import os           # 新增：导入 os
from glob import glob # 新增：导入 glob 用来匹配文件扩展名

package_name = 'my_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ==== 以下是你需要新增的 4 行代码，告诉编译器去哪里拷贝这些文件夹 ====
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.xml'))),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # ======================================================================
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='My first custom robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "four_wheel_filter = robot_control_system.four_wheel_filter_node:main",
        ],
    },
)