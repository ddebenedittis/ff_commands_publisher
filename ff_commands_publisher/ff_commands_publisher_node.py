import traceback

import numpy as np

from builtin_interfaces.msg import Time
import rclpy
import rclpy.duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import Player, PlayOptions
# from rosbag2_py._info import Info
from sensor_msgs.msg import JointState

from ff_commands_publisher.utils import get_bag_filepath



def sum_stamp(stamp: Time, seconds: float):
    """Add to a stamp object a number of seconds."""
    
    stamp.sec = stamp.sec + (stamp.nanosec + int(seconds * 10**9)) // 10**9
    stamp.nanosec = (stamp.nanosec + int(seconds * 10**9)) % 10**9
    
def gt(time1: Time, time2: Time):
    """Check whether time1 is greater than time2."""
    
    if time1.sec > time2.sec:
        return True
    if time1.sec == time2.sec and time1.nanosec > time2.nanosec:
        return True
    
    return False


class FFCommandsPublisher(Node):
    """Class that republishes the JointStates message recorded in a bag file."""
    
    def __init__(self):
        super().__init__('ff_commands_publisher')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameter('bag_filename', '010_move_base')
        bag_filename = str(self.get_parameter('bag_filename').get_parameter_value().string_value)
        bag_file_path = get_bag_filepath(bag_filename)
                
        self.declare_parameter('rate', 1)
        rate = self.get_parameter('rate').get_parameter_value().integer_value

        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 1)

        # Create the bag reader.
        self.reader = rosbag2_py.SequentialReader()
        storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_file_path, storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        
        # Extract the last JointState message from the bag.
        self.last_joint_state_msg = JointState()
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            if topic == '/joint_states':
                joint_state_msg: JointState = deserialize_message(data, JointState)
                if gt(joint_state_msg.header.stamp, self.last_joint_state_msg.header.stamp):
                    self.last_joint_state_msg = joint_state_msg
        self.last_joint_positions = np.array(self.last_joint_state_msg.position)
        
        # Homing phase at the end of the experiment.
        self.final_joint_positions = np.zeros(len(self.last_joint_positions))
        self.final_homing_time = 2 / rate
        
        # m: Info = Info().read_metadata(bag_file_path, 'mcap')
        # self.duration = m.duration.nanoseconds / 10**9
        self.duration = self.last_joint_state_msg.header.stamp.sec \
            + self.last_joint_state_msg.header.stamp.nanosec / 10**9
        
        # Reproduce the bag.
        player = Player()
        play_options = PlayOptions()
        play_options.rate = rate
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
            self.final_homing()
        else:
            self.get_logger().info("Finished the feed-forward trajectory.")
            raise KeyboardInterrupt()
        
    def final_homing(self):
        """Perform the final homing to bring the robot to the default configuration."""
        
        phi = self.time / self.final_homing_time
        sum_stamp(self.last_joint_state_msg.header.stamp, self.timer_period)
        
        positions = self.last_joint_positions + phi * (self.final_joint_positions - self.last_joint_positions)
        self.last_joint_state_msg.position = positions.tolist()
        
        self.pub_joint_states.publish(self.last_joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        
    try:
        ff_pub_node = FFCommandsPublisher()
        rclpy.spin(ff_pub_node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        rclpy.shutdown()
    except:
        traceback_logger.error(traceback.format_exc())
        
    ff_pub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
