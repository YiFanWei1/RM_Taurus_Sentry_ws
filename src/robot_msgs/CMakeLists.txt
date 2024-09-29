cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DecisionCtrl.msg"
  "msg/EnemyInfo.msg"
  "msg/GameStatus.msg"
  "msg/GimbalCtrl.msg"
  "msg/GimbalFdb.msg"
  "msg/LocationInfo.msg"
  "msg/RobotStatus.msg"
  "msg/VisionCtrl.msg"
  "msg/VisualStatus.msg"
  "msg/IsSub.msg"
  "msg/VisionStatus.msg"
  "srv/PidPlannerStatus.srv"
  "srv/Relocate.srv"
  "srv/Decision.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)


ament_package()
