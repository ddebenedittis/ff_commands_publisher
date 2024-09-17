# Feed-Forward Commands Publisher

ROS 2 package that publishes the robot's commands as a feed-forward control signal.

## Usage

The bags should be recorded with
```bash
ros2 bag record --use_sim_time /joint_states
```
and placed in the `bags` directory.

The syntax `[parameter_name:=parameter_value]` means that the parameter is optional. If you don't provide it, the default value will be used.

- Inspect the bag file (plots the joint positions, velocities, and efforts):
    ```bash
    ros2 run ff_commands_publisher bag_inspector
    ```
- Publish and visualize in RViz the SOLO12 motion:
    ```bash
    ros2 launch ff_commands_publisher visualize_solo.launch.py ["bag_filename:='<filename>'"] [rate:=<num>] [use_sim_time:=<true|false>] [topic_name:=<topic_name> default:='/PD_control/command']
    ```
    The outer "" in `"<param_name>:='<param_value>'"` are required only when the given filename is a number (e.g. 020), since we want it to be treated as a string.\
    - `"bag_filename:='<filename>'"`: select the bag to reproduce.
    - `rate`: increase or reduce the execution speed.
    - `topic_name`: remap `/joint_states` topic into another topic.
- Publish the bag JointState messages in the topic `/PD_control/command` (for using the controller on the real robot):
    ```bash
    ros2 launch ff_commands_publisher experiment_solo.launch.py ["bag_filename:='<filename>'"] [rate:=<float>]
    ```
