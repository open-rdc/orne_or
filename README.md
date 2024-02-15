# office_robot

### Install

Precondition:  
1) Install ROS noetic (Ubuntu20.04/docker)  
2) Install python3-catkin-tools
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install python3-catkin-tools
```

Install
```
cd ~/catkin_ws/src
git clone https://github.com/open-rdc/orne_or
wstool init
wstool merge orne_or/orne_or_pkgs.install
wstool up
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

### Execute

Navigation
Simulator (gazebo)  
```
roslaunch orne_or_bringup orne_or_sim.launch
roslaunch orne_or_navigation_executor nav_static_map.launch
```

Real Robot
```
roslaunch orne_or_bringup orne_or.launch
roslaunch orne_or_navigation_executor nav_static_map.launch
```

Manipulation
Simulator (gazebo)  
```
roslaunch orne_or_bringup orne_or_manipulator_sim.launch
roslaunch orne_or_moveit_config moveit_planner.launch
```

Real Robot
```
roslaunch orne_or_bringup orne_or_manipulator.launch
roslaunch orne_or_moveit_config moveit_planner.launch
```
