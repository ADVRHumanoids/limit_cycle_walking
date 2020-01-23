cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

project(limit_cycle_walking_tests C CXX)


find_package(cartesian_interface)
find_package(XBotInterface)

include_directories(${cartesian_interface_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS})

# compite GTest
find_package(GTest)
find_package(Threads)
if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()


add_definitions(-DWALKER_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")

set(TestLibs LimitCycleWalker ${GTEST_BOTH_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(TestSagittalPlane TestSagittalPlane.cpp)
target_link_libraries(TestSagittalPlane ${TestLibs})
add_dependencies(TestSagittalPlane GTest-ext LimitCycleWalker)
add_test(NAME TestSagittalPlane COMMAND TestSagittalPlane)

add_executable(TestLateralPlane TestLateralPlane.cpp)
target_link_libraries(TestLateralPlane ${TestLibs})
add_dependencies(TestLateralPlane GTest-ext LimitCycleWalker)
add_test(NAME TestLateralPlane COMMAND TestLateralPlane)

add_executable(TestEngine TestEngine.cpp)
target_link_libraries(TestEngine ${TestLibs})
add_dependencies(TestEngine GTest-ext LimitCycleWalker)
add_test(NAME TestEngine COMMAND TestEngine)

add_executable(TestParam TestParam.cpp)
target_link_libraries(TestParam ${TestLibs})
add_dependencies(TestParam GTest-ext LimitCycleWalker)
add_test(NAME TestParam COMMAND TestParam)



add_executable(TestWalker TestWalker.cpp)
target_link_libraries(TestWalker ${TestLibs}
                                 ${XBotInterface_LIBRARIES}
                                 ${cartesian_interface_LIBRARIES}
                                 )

add_dependencies(TestWalker GTest-ext LimitCycleWalker)
add_test(NAME TestWalker COMMAND TestWalker)
