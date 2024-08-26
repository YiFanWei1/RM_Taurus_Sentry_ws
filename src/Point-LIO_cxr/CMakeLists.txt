cmake_minimum_required(VERSION 3.5)
project(point_lio_cxr)
set(CMAKE_BUILD_TYPE "Debug")
# 设置C++标准版本
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
ADD_COMPILE_OPTIONS(-std=c++14)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(PCL_INCLUDE_DIRS /usr/include/pcl-1.12)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")

  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(PCL 1.12 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS} ${SENSOR_MSGS_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${pcl_conversions_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${pcl_conversions_DEFINITIONS})

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

# 设置编译选项
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()



# 添加可执行文件
add_executable(pointlio_mapping
  src/laserMapping.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/parameters.cpp
  src/preprocess.cpp 
  src/Estimator.cpp
)

# 链接所需库
ament_target_dependencies(pointlio_mapping
  rclcpp
  sensor_msgs
  nav_msgs 
  geometry_msgs
  tf2_ros
  tf2
  Eigen3
  PCL
  PythonLibs
  livox_ros_driver2
  visualization_msgs
  # eigen_conversions
  # matplotlibcpp
)

# 包含目录
target_include_directories(pointlio_mapping
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    livox_ros_driver2
    # ${matplotlibcpp_INCLUDE_DIRS}
)

target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} ${pcl_conversions_LIBRARIES} ${PYTHON_LIBRARIES} Eigen3::Eigen)

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

# if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
#   # Custommsg to support foxy & galactic
#   rosidl_target_interfaces(pointlio_mapping
#   pointlio_mapping "rosidl_typesupport_cpp")
# else()
#   rosidl_get_typesupport_target(cpp_typesupport_target
#   pointlio_mapping "rosidl_typesupport_cpp")
#   target_link_libraries(point     io_mapping ${cpp_typesupport_target})
# endif()

# 暴露接口
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  Eigen3
  PCL
  PythonLibs
  livox_ros_driver2
  # eigen_conversions
  # matplotlibcpp
)

# 安装
# install(TARGETS pointlio_mapping
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(TARGETS pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)


ament_package()