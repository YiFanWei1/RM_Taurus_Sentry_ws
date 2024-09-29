cmake_minimum_required(VERSION 3.8)
project(pid_follow_)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(Eigen3 REQUIRED)
# target_link_libraries(robot_base ${CERES_LIBRARIES})

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  # src/robot_base.cpp
  # src/crc.cpp
  # src/serial_device.cpp
  # src/filter.cpp
  # src/vel_calibrator.cpp
  src/pid_follow_.cpp

)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pid_follow_
  EXECUTABLE ${PROJECT_NAME}_node
)
target_link_libraries(pid_follow_ ${CERES_LIBRARIES})

#############
## Testing ##
#############

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_copyright
#     ament_cmake_uncrustify
#   )
#   ament_lint_auto_find_test_dependencies()
# endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  # include
  # include/pid_follow_
  src
#   config
  # launch
)
