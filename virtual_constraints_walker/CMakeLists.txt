cmake_minimum_required(VERSION 2.8.3)
project(virtual_constraints_walker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)




#find_package(XBotInterface REQUIRED)
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  std_srvs
  tf
  tf_conversions
  eigen_conversions
  cartesian_interface
)

find_package(XBotInterface REQUIRED)
find_package(OpenMpC REQUIRED)

# # Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#
# )

# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )


## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#
# )

generate_dynamic_reconfigure_options(
  cfg/gains.cfg
  cfg/walkingGains.cfg
)



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


add_library(virtual_constraints_walker STATIC
              src/virtualConstraintsNode.cpp
              src/robot_interface_ROS.cpp
              src/footStabilizer/footStabilizer.cpp
              src/footStabilizer/compute_cp)



target_link_libraries(virtual_constraints_walker ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES} ${OpenMpC_LIBRARIES})
 
#add_executable(VC_publisher src/VC_publisher.cpp)
#add_executable(VC_subscriber src/VC_subscriber.cpp)
#add_executable(VC_listener src/VC_listener.cpp)


add_executable(VC_run src/VC_run.cpp)

add_executable(compute_cp src/footStabilizer/compute_cp.cpp)


#target_link_libraries(VC_publisher ${catkin_LIBRARIES})
#target_link_libraries(VC_subscriber ${catkin_LIBRARIES} ${XBotInterface_LIBRARIES})
#target_link_libraries(VC_listener ${catkin_LIBRARIES})

target_link_libraries(VC_run virtual_constraints_walker ${catkin_LIBRARIES})

target_link_libraries(compute_cp virtual_constraints_walker ${catkin_LIBRARIES})
 
#add_dependencies(VC_publisher)
#add_dependencies(VC_subscriber)
#add_dependencies(VC_listener)
add_dependencies(compute_cp ${PROJECT_NAME}_gencfg)
add_dependencies(VC_run ${PROJECT_NAME}_gencfg)





#install(TARGETS VC_publisher 
#         RUNTIME DESTINATION lib/${PROJECT_NAME} #${CATKIN_PACKAGE_BIN_DESTINATION}
#         LIBRARY DESTINATION lib/${PROJECT_NAME})
#install(TARGETS VC_subscriber 
#         RUNTIME DESTINATION lib/${PROJECT_NAME} #${CATKIN_PACKAGE_BIN_DESTINATION}
#         LIBRARY DESTINATION lib/${PROJECT_NAME})
#install(TARGETS VC_listener 
#         RUNTIME DESTINATION lib/${PROJECT_NAME} #${CATKIN_PACKAGE_BIN_DESTINATION}
#         LIBRARY DESTINATION lib/${PROJECT_NAME})

install(TARGETS VC_run RUNTIME DESTINATION lib/${PROJECT_NAME}) #${CATKIN_PACKAGE_BIN_DESTINATION}
install(TARGETS compute_cp RUNTIME DESTINATION lib/${PROJECT_NAME}) #${CATKIN_PACKAGE_BIN_DESTINATION}