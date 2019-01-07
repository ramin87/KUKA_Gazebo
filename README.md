# Installation:

[Install ROS and Gazebo](http://gazebosim.org/tutorials?tut=ros_installing)

# Gazebo Environment along with the robot, gripper, camera and the object:
To bringup the environment:

```bash
roslaunch kuka_gripper_gazebo kuka_gripper_world.launch
roslaunch kuka_gripper_gazebo kuka_gripper_startup.launch 
=======
roslaunch my_gazebo myGripper.launch
```

![Alt text](./.Gazebo_env.png?raw=true "Title")



# Solved Problems:
[Solve the Grasp problem](https://github.com/JenniferBuehler/gazebo-pkgs/issues/9)


# Useful commands:
sudo apt-get install ros-indigo-joint-state-controller
sudo apt-get install ros-indigo-controller-manager
sudo apt-get install ros-indigo-joint-trajectory-controller
sudo apt-get install ros*controller*
sudo apt-get install ros-indigo-*controller*
sudo apt-get install ros-indigo-position-controllers

sudo apt-get install python-xlib

Gazebo ros package
