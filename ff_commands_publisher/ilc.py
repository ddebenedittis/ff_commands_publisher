import os
import traceback

import numpy as np

from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.executors
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ff_commands_publisher.utils import get_full_bag_filename



def multi_interp(x, xp, fp):
    f = np.zeros((x.shape[0], fp.shape[1]))
    for j in range(fp.shape[1]):
        f[:, j] = np.interp(x, xp, fp[:, j])
        
    return f

class ILC(Node):
    def __init__(self, experiment_name: str, order: int = 1) -> None:
        super().__init__('ilc')
        
        self.gain = 0.1
        self.order = order
        
        full_exp_name = get_full_bag_filename(experiment_name)
        workspace_directory = f"{get_package_share_directory('ff_commands_publisher')}/../../../../"
        self.ilc_dir = f"{workspace_directory}/ilc/{full_exp_name}"
        
        if not os.path.exists(self.ilc_dir):
            os.makedirs(self.ilc_dir)
        
        filenames = [f for f in os.listdir(self.ilc_dir) if f.endswith(".csv")]
        self.iteration = 1
        if not len(filenames) == 0:
            iteration_numbers = [int(fname.split('_')[-1].split('.')[0]) for fname in filenames]
            self.iteration = max(iteration_numbers) + 1
        
        self.command_subscr = self.create_subscription(
            JointState, '/PD_control/command', self.command_callback, 10
        )
        # self.command_subscr
        
        self.joint_state_subscr = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        # self.joint_state_subscr
        
        
        self.times_commands = np.zeros(0)
        self.commands = np.zeros((0, 12))
        
        self.times_joint_states = np.zeros(0)
        self.joint_states = np.zeros((0, 12))
        
    def get_ilc_command(self) -> np.ndarray:
        if self.iteration == 1:
            return None
        
        return np.loadtxt(f"{self.ilc_dir}/ilc_command_iteration_{self.iteration - 1}.csv", delimiter=',')
    
    def command_callback(self, msg: JointState) -> None:
        self.times_commands = np.append(
            self.times_commands, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
        self.commands = np.vstack((
            self.commands, np.array([msg.position])))
        
    def joint_state_callback(self, msg: JointState) -> None:
        self.times_joint_states = np.append(
            self.times_joint_states, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        
        self.joint_states = np.vstack((
            self.joint_states, np.array([msg.position])))
        
    def compute_error(self) -> np.ndarray:
        # joint_states_interp = multi_interp(
        #     self.times_commands, self.times_joint_states, self.joint_states)
        
        # return self.commands - joint_states_interp
        return self.commands
    
    def save_ilc_iteration(self) -> None:
        error = self.compute_error()

        if self.iteration == 1:
            np.savetxt(f"{self.ilc_dir}/times.csv", self.times_commands, delimiter=',')
            ilc_command = self.gain * error
        else:
            ilc_command = np.loadtxt(f"{self.ilc_dir}/ilc_command_iteration_{self.iteration - 1}.csv", delimiter=',')
            ilc_command = ilc_command + self.gain * error
            
        np.savetxt(f"{self.ilc_dir}/error_iteration_{self.iteration}.csv", error, delimiter=',')
        np.savetxt(f"{self.ilc_dir}/ilc_command_iteration_{self.iteration}.csv", ilc_command, delimiter=',')
        
        self.get_logger().info("Saved the ILC data of the current iteration.")
    