project(limit_cycle_walking)
cmake_minimum_required(VERSION 2.8.3)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


#find_package(XBotInterface REQUIRED)
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    cartesian_interface
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

add_service_files(
  FILES
  SetCommands.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
include_directories(${cartesian_interface_INCLUDE_DIRS})

include_directories(include/)



add_library(LimitCycleWalker STATIC
    src/engine/engine.cpp
    src/engine/lateral_plane.cpp
    src/engine/sagittal_plane.cpp
    src/engine/mpc_solver.cpp
    src/walker/foot_trajectory.cpp
    src/walker/walker.cpp
    src/param/param.cpp
    src/ros/walker_executor.cpp
    src/ros/walker_server.cpp)


target_link_libraries(LimitCycleWalker ${catkin_LIBRARIES}
                                       ${XBotInterface_LIBRARIES}
                                       ${OpenMpC_LIBRARIES}
                                       ${cartesian_interface_LIBRARIES}
                                       yaml-cpp)

install(TARGETS LimitCycleWalker LimitCycleWalker
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Walker Server executable
add_executable(walker_server src/ros/walker_server.cpp)
target_link_libraries(walker_server LimitCycleWalker)
install(TARGETS walker_server walker_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_definitions(-DWALKER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")


if(BUILD_TESTS)
    enable_testing()
    add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
    add_subdirectory(tests)
endif()
