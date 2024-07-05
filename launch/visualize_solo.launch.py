import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    solo_descr_share_path = FindPackageShare('solo_description')

    xacro_file_path = PathJoinSubstitution([
        solo_descr_share_path,
        LaunchConfiguration('xacro_file_path', default=os.path.join('xacro', 'solo12.urdf.xacro'))
    ])
    
    config_file_path = PathJoinSubstitution([
        solo_descr_share_path,
        LaunchConfiguration('config_file_path', default=os.path.join('config', 'rviz', 'standalone.rviz'))
    ])
    
    bag_filename = LaunchConfiguration('bag_filename', default='010_move_base')
    
    rate = LaunchConfiguration('rate', default='1')
    
    topic_name = LaunchConfiguration('topic_name', default='/joint_states')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ======================================================================== #

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': ParameterValue(Command(['xacro ', xacro_file_path]), value_type=str)}
            ],
        ),
        
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', config_file_path],
        ),
        
        Node(
            package='ff_commands_publisher',
            executable='ff_commands_publisher_node',
            name='ff_commands_publisher_node',
            parameters=[
                {'bag_filename': bag_filename},
                {'rate': rate},
                {'topic_name': topic_name},
            ],
            shell=True,
            emulate_tty=True,
            output = 'screen',
        ),
    ])
