cmake_minimum_required(VERSION 3.14)
project(vr_teleop LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ---- Common deps ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ---- PXREARobotSDK (vendored prebuilt .so) ----
set(PXREA_SDK_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PXREA_SDK_LIB     ${CMAKE_CURRENT_SOURCE_DIR}/lib/libPXREARobotSDK.so)
add_library(PXREARobotSDK SHARED IMPORTED GLOBAL)
set_target_properties(PXREARobotSDK PROPERTIES
  IMPORTED_LOCATION "${PXREA_SDK_LIB}"
  INTERFACE_INCLUDE_DIRECTORIES "${PXREA_SDK_INCLUDE}"
)

# ===========================================================================
# Target 1: teleop_openarm  (VR controller -> KDL IK -> joint commands)
# ===========================================================================
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(orocos_kdl_vendor QUIET)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(teleop_openarm src/teleop_openarm.cpp)
target_link_libraries(teleop_openarm
  PXREARobotSDK
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${control_msgs_TARGETS}
  orocos-kdl
  kdl_parser::kdl_parser
  ${urdf_TARGETS}
)

# ===========================================================================
# Target 2: video_sender  (ROS2 image topics -> H264 -> TCP -> VR headset)
# ===========================================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL  REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio highgui imgcodecs)
find_package(cv_bridge REQUIRED)

add_executable(video_sender src/video_sender.cpp)
target_compile_definitions(video_sender PRIVATE USE_ROS2)
target_include_directories(video_sender PRIVATE
  ${AVCODEC_INCLUDE_DIRS}
  ${AVUTIL_INCLUDE_DIRS}
  ${SWSCALE_INCLUDE_DIRS}
)
target_link_libraries(video_sender
  ${AVCODEC_LIBRARIES}
  ${AVUTIL_LIBRARIES}
  ${SWSCALE_LIBRARIES}
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  pthread
)

# ===========================================================================
# Install
# ===========================================================================
install(TARGETS teleop_openarm video_sender
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install bundled SDK .so so it ends up on RPATH via ament
install(FILES ${PXREA_SDK_LIB}
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Make installed binaries find libPXREARobotSDK.so in several common layouts:
# 1) the normal ROS install tree,
# 2) executable copied next to the .so,
# 3) executable copied with a sibling ../lib directory.
# Also search the default ROS 2 Humble runtime path for core ROS libraries.
set_target_properties(teleop_openarm PROPERTIES
  INSTALL_RPATH "/opt/ros/humble/lib;$ORIGIN;$ORIGIN/../lib;$ORIGIN/../../lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

ament_package()
