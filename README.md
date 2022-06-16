# PointOneNav Atlas ROS2 Driver

FusionEngine delivers 10cm location accuracy in any environment using its proprietary sensor fusion algorithm.

### Getting Started

##### Install driver dependencies:
```
$ sudo apt-get update
$ sudo rosdep update
$ rosdep install -y --ignore-src --from-paths .
```

##### Deploy Atlas node
```
$ source /opt/ros/galactic/setup.bash
$ colcon build
$ source install/local_setup.bash
$ ros2 run atlas_driver atlas_driver_node_exe
```
