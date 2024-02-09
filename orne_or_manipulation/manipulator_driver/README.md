# manipulator_driver

## Overview
Driver for controlling manipulator
ROS control

## Usage
```
$ roslaunch manipulator_driver driver.launch
```

## ROS API
### subscribed topics
none

### published topics
* `/position` (std_msgs/Float32MultiArray)

### services
service for B3M servo motors
* `/manipulator_driver/<B3M driver name>/set_power` (manipulator_driver/setPower)
  * servo ON
  * clear error message
