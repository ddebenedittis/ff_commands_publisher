# Feed-Forward Commands Publisher

ROS 2 package that publishes the robot's commands as a feed-forward control signal.

## Usage

The bags should be recorded with
```bash
ros2 bag record --use_sim_time /joint_states
```
and placed in the `bags` directory.

- Inspect the bag file (plots the joint positions, velocities, and efforts):
```bash
ros2 run ff_commands_publisher bag_inspector
```
- Publish and visualize in RViz SOLO12 motion:
```bash
ros2 launch ff_commands_publisher visualize_solo.launch.py ["bag_filename:='<filename>'"] [rate:=<num>] [use_sim_time:=<true|false>] 
```
- Publish the bag JointState messages in the topic `/joint_states`:
```bash
ros2 launch ff_commands_publisher ff_commands_publisher_node.launch.py ["bag_filename:='<filename>'"]
```
