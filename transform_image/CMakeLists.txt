cmake_minimum_required(VERSION 3.0.2)
project(transform_image)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  rospy
  std_msgs
)

# catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})