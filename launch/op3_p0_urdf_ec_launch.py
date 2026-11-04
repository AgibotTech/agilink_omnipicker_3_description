from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    package_share = Path(get_package_share_directory('omnipicker_3_description'))
    model_path = package_share / 'urdf' / 'xacro' / 'op3_p0_urdf_ec.xacro'
    robot_description = ParameterValue(
        xacro.process_file(str(model_path)).toxml(), value_type=str
    )
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool
    )
    model_parameters = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the single joint1 gripper slider.'
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 with the OP3 P0 view.'
        ),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true',
            description='Publish preview joint states; disable for hardware or simulation.'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the simulation clock.'
        ),
        DeclareLaunchArgument(
            'initial_position', default_value='0.0',
            description='Initial joint1 stroke in meters.'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[model_parameters, {'publish_frequency': 50.0}],
        ),
        Node(
            package='omnipicker_3_description',
            executable='gripper_control.py',
            name='gripper_joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_joint_states')),
            parameters=[{
                'urdf_path': str(package_share / 'urdf' / 'OP3_P0_URDF_EC.urdf'),
                'use_sim_time': use_sim_time,
                'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
                'initial_position': ParameterValue(
                    LaunchConfiguration('initial_position'), value_type=float
                ),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(package_share / 'rviz' / 'op3_p0_urdf_ec.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
