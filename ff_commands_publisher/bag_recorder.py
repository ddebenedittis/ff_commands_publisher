from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import rosbag2_py
from rosbag2_py import Recorder, RecordOptions

from ff_commands_publisher.utils import get_full_bag_filename



class BagRecorder(Node):
    """Utulity node to inspect the content of the /joint_states meccases in a bag file."""
    
    def __init__(self):
        super().__init__('BagRecorder')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameter('time', 'yyyy-mm-dd-hh-mm-ss')
        time = str(self.get_parameter('time').get_parameter_value().string_value)
        
        self.declare_parameter('bag_filename', '010_move_base')
        workspace_directory = f"{get_package_share_directory('ff_commands_publisher')}/../../../../"
        bag_filename = str(self.get_parameter('bag_filename').get_parameter_value().string_value)
        bag_full_filename = get_full_bag_filename(bag_filename)
        bag_filepath = f"{workspace_directory}/bags/{time}-{bag_full_filename}"
        
                
        # Create the bag reader.
        self.recorder = Recorder()
        
        self.record_options = RecordOptions()
        self.record_options.all = True
        
        self.storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_filepath, storage_id='mcap')
        
        self.get_name()
        
    def __call__(self):
        try:
            self.recorder.record(self.storage_options, self.record_options)
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    recorder = BagRecorder()
    recorder()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
