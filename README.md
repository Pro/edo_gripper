e.DO Gripper
============



To start gazebo with the dummy gripper use:

```sh
roslaunch edo_gripper gazebo_dummy_gripper.launch

# Set gripper span. The value is in meter
rostopic pub /edo_gripper/set_gripper_span std_msgs/Float32 "data: 0.05"
```