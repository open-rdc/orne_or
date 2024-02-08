cmake_minimum_required(VERSION 2.8.3)
project(manipulator_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  message_generation
  controller_manager
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  MotorParameter.msg
  MotorParameters.msg
)

add_service_files(
  FILES
  setPower.srv
  SetMotorProfiles.srv
  SetTorqueCoefficients.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(manipulator_driver src/manipulator_driver.cpp src/b3m.c)
add_dependencies(manipulator_driver ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(manipulator_driver ${catkin_LIBRARIES} usb ftdi)
