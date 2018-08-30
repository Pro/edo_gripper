e.DO Gripper
============

## Testing with RViz

To test the URDF in RViz use the two commands:

```
roslaunch edo_gripper edo_gripper_dummy_upload.launch
roslaunch edo_gripper test.launch
```

## Testing with Gazebo

To start gazebo with the dummy gripper use:

```sh
roslaunch edo_gripper edo_gripper_dummy.launch 

# Set gripper span. The value is in meter
rostopic pub /edo/set_gripper_span std_msgs/Float32 "data: 0.05"
```