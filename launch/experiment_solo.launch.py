from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
        
    bag_filename = LaunchConfiguration('bag_filename', default='010_move_base')
    
    rate = LaunchConfiguration('rate', default='1.0')
    
    workspace_path = f"{get_package_share_directory('solo12_sim')}/../../../../"
    time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    output_bag_path = f"{workspace_path}/bags/{time}"
    
    print(f"output_bag_path: {output_bag_path}")
        
    # ======================================================================== #
    
    ff_commands_pub = Node(
        package='ff_commands_publisher',
        executable='ff_commands_publisher_node',
        name='ff_commands_publisher_node',
        parameters=[
            {'bag_filename': bag_filename},
            {'rate': rate},
            {'topic_name': '/PD_control/command'},
        ],
        shell=True,
        emulate_tty=True,
        output = 'screen',
    )
    
    bag_recorder = Node(
        package='ff_commands_publisher',
        executable='bag_recorder',
        name='bag_recorder',
        parameters=[
            {'bag_filename': bag_filename},
            {'rate': rate},
            {'time': time},
        ],
        shell=True,
        emulate_tty=True,
        output = 'screen',
    )

    return LaunchDescription([
        bag_recorder,
        
        ff_commands_pub,
    ])
