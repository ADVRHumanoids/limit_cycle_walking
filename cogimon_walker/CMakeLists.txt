cmake_minimum_required(VERSION 2.8)
project(cogimon_walker)


add_compile_options("-std=c++11")


add_subdirectory(opensot/)
add_subdirectory(VC_cartesio/)


#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  FILES_MATCHING PATTERN "*.launch"
#)

#install(DIRECTORY cartesio/problems/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cartesio/problems
#  FILES_MATCHING PATTERN "*.yaml"
#)

#install(DIRECTORY robots/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
#)
