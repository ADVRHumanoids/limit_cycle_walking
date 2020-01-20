cmake_minimum_required(VERSION 2.8.3)
project(limit_cycle_walking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)




#find_package(XBotInterface REQUIRED)
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
  std_srvs
  tf
  tf_conversions
  eigen_conversions
)

find_package(XBotInterface REQUIRED)
find_package(OpenMpC REQUIRED)


catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   
)


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${XBotInterface_INCLUDE_DIRS})
include_directories(include/)


add_library(LimitCycleWalker STATIC
              src/virtualConstraintsNode.cpp
              src/walker/walker.cpp
              src/walker/lateral_plane.cpp
              src/walker/sagittal_plane.cpp
              src/step_machine/foot_trajectory.cpp
              src/step_machine/step_machine.cpp
              src/robot_interface_ROS.cpp)


target_link_libraries(LimitCycleWalker ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} ${OpenMpC_LIBRARIES})
 
add_executable(VC_run src/VC_run.cpp)

target_link_libraries(VC_run LimitCycleWalker ${catkin_LIBRARIES})

install(TARGETS VC_run RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()
