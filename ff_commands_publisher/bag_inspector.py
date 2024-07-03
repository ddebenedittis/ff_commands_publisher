import matplotlib.pyplot as plt
import numpy as np

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import JointState

from ff_commands_publisher.utils import get_bag_filepath



class InspectBag(Node):
    """Utulity node to inspect the content of the /joint_states meccases in a bag file."""
    
    def __init__(self):
        super().__init__('inspect_bag')
        
        # ============================ Parameters ============================ #
        
        self.declare_parameter('bag_filename', '010_move_base')
        bag_filename = str(self.get_parameter('bag_filename').get_parameter_value().string_value)
        bag_file_path = get_bag_filepath(bag_filename)
                
        # Create the bag reader.
        self.reader = rosbag2_py.SequentialReader()
        storage_options: rosbag2_py.StorageOptions = rosbag2_py._storage.StorageOptions(
            uri=bag_file_path, storage_id='mcap')
        converter_options: rosbag2_py.ConverterOptions = rosbag2_py._storage.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        
        # Count the number of joint_states messages in order to initialize the arrays.
        counter, n_joints = self._count_messages(storage_options, converter_options)
        
        self.time = np.zeros((counter))
        self.joint_pos = np.zeros((counter, n_joints))
        self.joint_vel = np.zeros((counter, n_joints))
        self.effort = np.zeros((counter, n_joints))
        
    def _count_messages(self, storage_options, converter_options) -> tuple[int, int]:
        """
        Count the number of joint_states messages in the bag file.

        Args:
            storage_options: _description_
            converter_options: _description_

        Returns:
            tuple[int, int]: The number of joint_states messages and the number of joints.
        """
        
        counter = 0
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            if topic == '/joint_states':
                if counter == 0:
                    joint_state_msg: JointState = deserialize_message(data, JointState)
                    n_joints = len(joint_state_msg.position)
                    
                counter += 1
        self.reader.open(storage_options, converter_options)    # reopen the reader.
        
        return counter, n_joints
        
    def _plot(self):
        fig, axs = plt.subplots(3, 1)
        fig.tight_layout()
        
        for ax in axs:
            ax.autoscale(enable=True, axis='x', tight=True)
            ax.grid(True)
        
        axs[0].plot(self.time, self.joint_pos)
        axs[0].set(
            xlabel='Time [s]',
            ylabel='Position [rad]',
            title='Joint Positions'
        )
        
        axs[1].plot(self.time, self.joint_vel)
        axs[1].set(
            xlabel='Time [s]',
            ylabel='Velocity [rad/s]',
            title='Joint Velocities'
        )
        
        axs[2].plot(self.time, self.effort)
        axs[2].set(
            xlabel='Time [s]',
            ylabel='Effort [Nm]',
            title='Joint Efforts'
        )
        
        plt.show()
            
    def __call__(self):
        counter = 0
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()
            if topic == '/joint_states':
                joint_state_msg: JointState = deserialize_message(data, JointState)
                
                self.time[counter] = joint_state_msg.header.stamp.sec \
                    + joint_state_msg.header.stamp.nanosec * 1e-9
                self.joint_pos[counter, :] = joint_state_msg.position
                self.joint_vel[counter, :] = joint_state_msg.velocity
                self.effort[counter, :] = joint_state_msg.effort
                
                counter += 1
                    
        self._plot()


def main(args=None):
    rclpy.init(args=args)
    
    inspector = InspectBag()
    inspector()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
