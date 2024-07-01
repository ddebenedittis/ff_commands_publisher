import numpy as np

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import Player, PlayOptions
# from rosbag2_py._info import Info
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState


def sum_stamp(stamp: Time, seconds: float):
    stamp.sec = stamp.sec + (stamp.nanosec + int(seconds * 10**9)) // 10**9
    stamp.nanosec = (stamp.nanosec + int(seconds * 10**9)) % 10**9
    
def gt(time1: Time, time2: Time):
    if time1.sec > time2.sec:
        return True
    if time1.sec == time2.sec and time1.nanosec > time2.nanosec:
        return True
    
    return False


class FFCommandsPublisher(Node):
    def __init__(self, bag_file_path):
        super().__init__('ff_commands_publisher')

        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 1)

        self.reader = rosbag2_py.SequentialReader()
        storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_file_path, storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        
        # Extract the last JointState message from the bag.
        self.joint_state_msg = JointState()
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            if topic == '/joint_states':
                joint_state_msg: JointState = deserialize_message(data, JointState)
                if gt(joint_state_msg.header.stamp, self.joint_state_msg.header.stamp):
                    self.joint_state_msg = joint_state_msg
        self.joint_positions = np.array(self.joint_state_msg.position)
        
        self.final_joint_positions = np.zeros(len(self.joint_positions))
        self.final_homing_time = 2
        
        # m: Info = Info().read_metadata(bag_file_path, 'mcap')
        # self.duration = m.duration.nanoseconds / 10**9
        self.duration = self.joint_state_msg.header.stamp.sec \
            + self.joint_state_msg.header.stamp.nanosec / 10**9
        
        # Reproduce the bag.
        player = Player()
        play_options = PlayOptions()
        play_options.rate = 1
        try:
            player.play(storage_options, play_options)
        except KeyboardInterrupt:
            pass
        
        self.timer_period = 1.0 / 300  # seconds
        self.time = 0
        self.timer = self.create_timer(self.timer_period, self.publish_joint_states)
    
    def publish_joint_states(self):
        self.time += self.timer_period
        if self.time < self.final_homing_time:
            phi = self.time / self.final_homing_time
            sum_stamp(self.joint_state_msg.header.stamp, self.timer_period)
            
            positions = self.joint_positions + phi * (self.final_joint_positions - self.joint_positions)
            self.joint_state_msg.position = positions.tolist()
            
            self.pub_joint_states.publish(self.joint_state_msg)
        else:
            pass


def main(args=None):
    rclpy.init(args=args)
    package_share_directory = get_package_share_directory('ff_commands_publisher')
    bag_file_path = package_share_directory + "/bags/010_move_base/010_move_base.mcap"
    
    ff_pub_node = FFCommandsPublisher(bag_file_path)
    
    try:
        rclpy.spin(ff_pub_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        rclpy.shutdown()
        
    ff_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
