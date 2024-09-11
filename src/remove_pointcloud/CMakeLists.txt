cmake_minimum_required(VERSION 3.8)
project(remove_pointcloud)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")
#######################
## Find dependencies ##
#######################
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIRS})
find_package(ament_cmake_auto REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Num.msg"
#   # "srv/AddThreeInts.srv"
#  )
ament_auto_find_build_dependencies()
# find_package(Eigen3 REQUIRED)
# target_link_libraries(robot_base ${CERES_LIBRARIES})

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/remove_pointcloud.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN taurus::RemovePointcloud
  EXECUTABLE ${PROJECT_NAME}_node
)
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN StaticTransformPublisher
#   EXECUTABLE test_node
# )
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN TransformListenerNode
#   EXECUTABLE tf_listener_node
# )
# target_link_libraries(cxr_localization_ros2 ${CERES_LIBRARIES})

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
  include
  src
  config
  launch
)
