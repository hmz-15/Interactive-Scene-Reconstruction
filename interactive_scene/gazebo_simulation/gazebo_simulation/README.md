# Gazebo Simulation

We also provide interface to generate a Gazebo simulation environment of the reconstructed scene, which could be directly loaded into the Gazebo simulator, and supports robot perception, navigation and manipulation tasks.

For details of generating the Gazebo simulation world, please visit [HERE](https://github.com/hmz-15/Interactive-Scene-Reconstruction/tree/main/interactive_scene#31-gazebo-scene-generation).

## 1. Installation

### 1.1 Dependencies

For running our demo experiments, several ROS packages need to be installed beforehand.

- [gazebo_ros](http://wiki.ros.org/gazebo_ros)
- [fetch_gazebo](https://github.com/fetchrobotics/fetch_gazebo) (optional)
- [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) (optional)

You can use following command the install these packages
```bash
# replace the <ros-distro> with your own ROS distribution
# e.g., ros-melodic-fetch-gazebo-demo
sudo apt install ros-<ros-distro>-fetch-gazebo-demo
sudo apt install ros-<ros-distro>-gazebo-ros
sudo apt install ros-<ros-distro>-teleop-twist-keyboard
```

Noted that `fetch_gazebo` and `teleop_twist_keyboard` are not mandatory, you can launch the simulation with your own robot configurations, just make sure they have gazebo module included/supported.

## 2. Usage

Use following command to **launch the gazebo simulation world**.

```bash
roslaunch roslaunch gazebo_simulation gazebo_world.launch
```

If you do not want to spawn the robot, set `enable_robot` to `false`. For more configuration setups, check the launch file [gazebo_world.launch](launch/gazebo_world.launch).



Use following command to teleoperate the robot via keyboard & visualize the robot state in rviz

```bash
roslaunch gazebo_simulation fetchbot_teleop.launch
```





















