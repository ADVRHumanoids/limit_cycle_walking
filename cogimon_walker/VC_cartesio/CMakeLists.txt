project(VC_cartesio)
cmake_minimum_required(VERSION 3.0.0)

find_package(cartesian_interface REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

include_directories(controllers/ 
                    ../opensot/include/ 
                    ${cartesian_interface_INCLUDE_DIRS})

# add_library(CartesianVcIk controllers/IkController_VC.cpp)
# target_link_libraries(CartesianVcIk ${cartesian_interface_LIBRARIES} IKProblem)

# ycm_library_install(CartesianVcIk 1 0 0)
