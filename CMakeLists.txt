cmake_minimum_required(VERSION 3.5)
project(helipr_file_player)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set default build type to 'Release' if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find ament and other required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Gui OpenGL)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(novatel_gps_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Specify the include directories
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Set source files
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(FILE_PLAYER_QTLib_src ${SRC_DIR}/mainwindow.cpp ${SRC_DIR}/ROSThread.cpp)
set(FILE_PLAYER_QTLib_hdr ${SRC_DIR}/mainwindow.h ${SRC_DIR}/ROSThread.h)
set(FILE_PLAYER_QTLib_ui ${SRC_DIR}/mainwindow.ui)
set(FILE_PLAYER_QTBin_src ${SRC_DIR}/main.cpp)

qt5_wrap_ui(FILE_PLAYER_QTLib_ui_moc ${FILE_PLAYER_QTLib_ui})
qt5_wrap_cpp(FILE_PLAYER_QTLib_hdr_moc ${FILE_PLAYER_QTLib_hdr})
qt5_add_resources(SHADER_RSC_ADDED ${SRC_DIR}/resources.qrc)

# Define executable
add_executable(helipr_file_player ${FILE_PLAYER_QTLib_src} ${FILE_PLAYER_QTLib_hdr} ${FILE_PLAYER_QTBin_src} ${SHADER_RSC_ADDED} ${FILE_PLAYER_QTLib_ui_moc} ${FILE_PLAYER_QTLib_hdr_moc})

# Specify dependencies for the executable
ament_target_dependencies(helipr_file_player
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  PCL
  pcl_conversions
  tf2
  Eigen3
  std_srvs
  rosbag2_cpp
  rosgraph_msgs
  novatel_gps_msgs
  livox_ros_driver2
)

# Link libraries
target_link_libraries(helipr_file_player
  Qt5::Widgets
  Qt5::Gui
  Qt5::OpenGL
  GL
  ${rclcpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# Install the executable
install(TARGETS helipr_file_player
  DESTINATION lib/${PROJECT_NAME}
)

# Install message files (ROS 2)
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME} 
)

ament_package()
