import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # เริ่มต้น robot.py เป็น service
        Node(
            package='natthawat_final',  # แก้เป็นชื่อแพ็กเกจจริง
            executable='robot',  # ต้องตรงกับชื่อที่กำหนดใน setup.py
            name='robot',
            output='screen',
        ),
        # เริ่มต้น controller.py เป็น client
        Node(
            package='natthawat_final',
            executable='controller',  # ต้องตรงกับชื่อที่กำหนดใน setup.py
            name='controller',
            output='screen',
        ),
    ])
