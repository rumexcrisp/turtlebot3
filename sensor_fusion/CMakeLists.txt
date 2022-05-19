cmake_minimum_required(VERSION 3.0.2)

project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS 
  dynamic_reconfigure
  roscpp
  sensor_msgs
  laser_geometry
  tf
  pcl_ros
  pcl_conversions
)

generate_dynamic_reconfigure_options(
  cfg/fusion.cfg
)

catkin_package()

add_executable(${PROJECT_NAME} src/sensor_fusion.cpp src/main.cpp)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(DIRECTORY config launch
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )