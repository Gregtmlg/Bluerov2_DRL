cmake_minimum_required(VERSION 3.0.2)
project(bluerov_sim)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  apriltag_ros
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

add_message_files(
    FILES
    ActuatorCommands.msg
    RangeMeasurement.msg
    RangeMeasurementArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_install_python(PROGRAMS
  nodes/keyboard_control.py
  nodes/mixer.py
  nodes/range_sensor_node.py
  nodes/rviz_visualizer.py
  nodes/auto_arm_node.py
  nodes/velodyne_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
      roscpp
      gazebo_ros
  CATKIN_DEPENDS
    message_runtime
)

add_library(gazebo_pressure_plugin plugins/gazebo_pressure_plugin.cpp)
add_dependencies(gazebo_pressure_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_pressure_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_range_sensor_plugin plugins/gazebo_range_sensor_plugin.cpp)
add_dependencies(gazebo_range_sensor_plugin ${catkin_EXPORTED_TARGETS} bluerov_sim_generate_messages_cpp)
target_link_libraries(gazebo_range_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
