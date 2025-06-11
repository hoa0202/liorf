cmake_minimum_required(VERSION 3.5)
project(liorf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

# 헤더 프리컴파일 활성화
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++17 COMPILER_SUPPORTS_CXX17)
option(USE_PCH "Use precompiled headers" ON)

# 빌드 시간 개선 설정
if(USE_PCH)
  message(STATUS "Using precompiled headers")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Winvalid-pch")
  # ccache 사용 설정
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    message(STATUS "Using ccache to speed up compilation")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  endif()
endif()

# 링커 최적화 옵션 추가
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)  # 추가된 부분
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

# SQLite3 라이브러리 찾기
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

set(ros2_lib "rclcpp;rclpy;std_msgs;sensor_msgs;geometry_msgs;nav_msgs;cv_bridge;pcl_msgs;pcl_conversions;visualization_msgs;tf2;tf2_ros;tf2_eigen;tf2_sensor_msgs;tf2_geometry_msgs;pcl_ros")  # pcl_ros 추가

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  "msg/CloudInfo.msg"
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# ROS2 버전에 따른 처리
if(${rosidl_cmake_VERSION} VERSION_LESS "2.5.0")
  set(cpp_typesupport_target "${PROJECT_NAME}__rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

# include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  lib
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${pcl_ros_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
)

# link directories
link_directories(
  lib
  include
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${SQLITE3_LIBRARY_DIRS}
)

file(GLOB common_lib "lib/*.cpp")

###########
## Build ##
###########
# <!-- liorf_yjz_lucky_boy -->
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imageProjection ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization Library
add_library(${PROJECT_NAME}_mapOptimization_lib SHARED
  src/mapOptimization.cpp 
  src/costmap.cpp 
  src/loopclosure.cpp
  src/db_manager.cpp
  ${common_lib} 
  include/Scancontext.cpp
)
target_compile_options(${PROJECT_NAME}_mapOptimization_lib PRIVATE ${OpenMP_CXX_FLAGS})
if(USE_PCH)
  target_precompile_headers(${PROJECT_NAME}_mapOptimization_lib PRIVATE 
    <pcl/point_cloud.h>
    <pcl/point_types.h>
    <gtsam/geometry/Pose3.h>
    <Eigen/Dense>
  )
endif()
ament_target_dependencies(${PROJECT_NAME}_mapOptimization_lib ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_mapOptimization_lib Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} ${GeographicLib_LIBRARIES} gtsam ${SQLITE3_LIBRARIES})

# Mapping Optimization Executable
add_executable(${PROJECT_NAME}_mapOptimization src/main.cpp)
target_link_libraries(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME}_mapOptimization_lib)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization ${ros2_lib})

# # IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp ${common_lib})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration ${ros2_lib})
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer "${cpp_typesupport_target}" ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(grid_map_generator src/grid_map_generator.cpp)
ament_target_dependencies(grid_map_generator
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(grid_map_generator
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS grid_map_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME})


# Removert Node: 동적 물체 제거용 노드
add_executable(removert_node
  src/removert_node.cpp
  include/tsdf_grid.cpp    # 실제 경로로 수정
)
ament_target_dependencies(removert_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)
target_link_libraries(removert_node
  ${PCL_LIBRARIES}
)
install(TARGETS removert_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS 
  ${PROJECT_NAME}_imageProjection
  ${PROJECT_NAME}_imuPreintegration
  ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}_mapOptimization_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()