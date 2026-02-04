import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'mpc_planner'
    urdf_name = 'fishbot_gazebo.urdf'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'urdf', urdf_name)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')
    goal_file_path = os.path.join(pkg_share, 'goal', 'goal.txt')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    simulator_node = Node(
        package='mpc_planner',
        executable='simple_simulator',
        name='simple_simulator',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate': 50.0,
            'cmd_timeout': 0.5
        }]
    )

    mpc_controller_node = Node(
        package='mpc_planner',
        executable='mpc_controller',
        name='mpc_controller',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'mpc_params.yaml')]
    )

    goal_sender_node = Node(
        package='mpc_planner',
        executable='goal_sender',
        name='goal_sender',
        output='screen',
        parameters=[{'goal_file': goal_file_path}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(simulator_node)
    ld.add_action(mpc_controller_node)
    ld.add_action(goal_sender_node)
    ld.add_action(rviz_node)
    return ld