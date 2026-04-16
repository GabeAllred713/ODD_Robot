from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'device_type:=D405']
            arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'serial_no:=_335122272231', '-r', '/arm/D405/color/image_raw:=/arm/D405/color/image_rect_raw', '-p', 'pointcloud.enable:=true', '-p',  'align_depth.enable:=true', '-p', 'spatial_filter.enable:=true', '-p', 'hole_filling_filter.enable:=true', '-p', 'decimation_filter.enable:=true']#
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'device_type:=D435']
            arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'serial_no:=_207222073869', '-r', '/odd/D435/color/image_raw:=/odd/D435/color/image_rect_raw']
        ),
        Node(
            package='topic_tools',
            executable='mux',
            name='input_image_mux',
            namespace='/nanoowl',
            arguments=['/nanoowl/input_image', '/odd/D435/color/image_rect_raw', '/arm/D405/color/image_rect_raw']
        ),
        Node(
            package='topic_tools',
            executable='mux',
            name='depth_image_mux',
            namespace='/odd_arm_cv',
            arguments=['/odd_arm_cv/selected_depth_image', '/odd/D435/depth/image_rect_raw', '/arm/D405/aligned_depth_to_color/image_raw']
        ),
        Node(
            package='odd_arm_cv',
            executable='gui'
        ),
        #ExecuteProcess(
        #    cmd=['jetson-containers', 'run', '--ipc=host', '--pid=host', 'odd_arm/nanoowl:1.0.0 bash /opt/odd_arm_cv/nanoowl.sh'],
        #    output='screen'
        #)
    ])
