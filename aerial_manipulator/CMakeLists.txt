# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(aerial-manipulator)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  sensor_msgs 
  nav_msgs 
  mav_msgs
)

find_package(Eigen3 REQUIRED)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teensy_comunication
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs mav_msgs
#  DEPENDS system_lib
)





include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR} )



add_executable( UAM4_dynamics_control_sim_complete 
~/UAV/src/aerial_manipulator/src/UAM4_dynamics_control_sim_complete.cpp )
target_link_libraries( UAM4_dynamics_control_sim_complete 
${catkin_LIBRARIES})


add_executable( UAM4_dynamics_control_sim_static 
~/UAV/src/aerial_manipulator/src/UAM4_dynamics_control_sim_static.cpp )
target_link_libraries( UAM4_dynamics_control_sim_static
${catkin_LIBRARIES})


