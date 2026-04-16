from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odd_package',
            executable='control',
            namespace='odd'
            # arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'device_type:=D405']
            # arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'serial_no:=_335122272231']
        ),
        Node(
            package='odd_package',
            executable='gui',
            namespace='odd'
            # arguments=['--ros-args', '--log-level', 'DEBUG']
            # arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'device_type:=D435']
            # arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'serial_no:=_207222073869', '-r', '/odd/D435/color/image_raw:=/odd/D435/color/image_rect_raw']
        )
    ])
