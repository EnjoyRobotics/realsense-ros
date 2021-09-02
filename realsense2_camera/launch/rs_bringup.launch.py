import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, PythonExpression

from typing import Text
import xacro

counter = 0

def generate_launch_description():

	# args
	arg_use_sim_time = LaunchConfiguration('use_sim_time')

	output = 'screen'

	realsense_cam_parameters = os.path.join(get_package_share_directory('realsense2_camera'), 'config', 'realsense.yaml')

	t265_camera = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='t265_camera',
		namespace='t265_camera',
		emulate_tty=True,
		prefix=['stdbuf -o L'],
		parameters=[realsense_cam_parameters, {'use_sim_time':arg_use_sim_time}],
		remappings=[('odom/sample', 'odom')],
	)

	d435_camera1 = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='d435_camera1',
		namespace='d435_camera1',
		emulate_tty=True,
		prefix=['stdbuf -o L'],
		parameters=[realsense_cam_parameters],
		#  parameters=[realsense_cam_parameters, {'use_sim_time':arg_use_sim_time, 'serial_number': '843112070385'}],
	)

	d435_camera2 = Node(
		package='realsense2_camera',
		executable='realsense2_camera_node',
		name='d435_camera2',
		namespace='d435_camera2',
		emulate_tty=True,
		prefix=['stdbuf -o L'],
		parameters=[realsense_cam_parameters],
		#  parameters=[realsense_cam_parameters, {'use_sim_time':arg_use_sim_time, 'serial_number': '913422070791'}],
	)


	def start_t265_camera(text):
		global counter
		if 'RealsenseNode Is Up' in str(text):
			counter += 1
		if counter == 2:
			counter = 3 # launch only once
			ld = LaunchDescription([t265_camera])
			return ld
		else:
			return

	ld = LaunchDescription(
		# args
		[
			DeclareLaunchArgument('use_sim_time',	  default_value='false'),
			d435_camera1,
			d435_camera2,

            RegisterEventHandler(
            event_handler=OnProcessIO(
                on_stderr=lambda info: start_t265_camera(info.text)
            )),
        ]
	)

	return ld
