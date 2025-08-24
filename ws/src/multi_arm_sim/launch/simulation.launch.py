import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_arm_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'print_world.sdf')

    return LaunchDescription([
        # Launch Gazebo with print world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Launch MoveIt for motion planning
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{'robot_description': os.path.join(pkg_share, 'urdf', 'xarm_lite.urdf')}]
        ),
    ])
