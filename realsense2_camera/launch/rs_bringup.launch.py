import os
from distutils.util import strtobool

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessIO

from launch_ros.actions import Node

counter = 1


def generate_launch_description() -> LaunchDescription:
    simulation = bool(strtobool(os.environ.get('SIMULATION')))

    realsense_cam_parameters = os.path.join(get_package_share_directory('realsense2_camera'), 'config', 'realsense.yaml')

    preset_file = os.path.join(get_package_share_directory('realsense2_camera'), 'config', 'high_accuracy_preset.json')

    t265_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='t265_camera',
        namespace='t265_camera',
        emulate_tty=True,
        prefix=['stdbuf -o L'],
        parameters=[realsense_cam_parameters, {'use_sim_time': simulation}],
        remappings=[('odom/sample', 'odom')],
        condition=UnlessCondition(str(simulation)),
    )

    d435_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435_camera',
        namespace='d435_camera',
        emulate_tty=True,
        prefix=['stdbuf -o L'],
        parameters=[realsense_cam_parameters, {'json_file_path': preset_file, 'use_sim_time': simulation}],
        remappings=[('/d435_camera/depth/color/points', '/cloud_in')],
        condition=UnlessCondition(str(simulation)),
    )

    def start_t265_camera(text: str) -> type(None):
        global counter
        if 'RealSense Node Is Up' in str(text):
            counter += 1
        if counter == 2:
            counter = 3  # launch only once
            ld = LaunchDescription([t265_camera])
            return ld
        else:
            return

    ld = LaunchDescription(
        # args
        [
            d435_camera,

            RegisterEventHandler(event_handler=OnProcessIO(
                on_stderr=lambda info: start_t265_camera(info.text)
            )),
        ]
    )

    return ld
