cmake_minimum_required(VERSION 3.5)
project(ximea_usb_ros_driver)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(cmake/FindXIMEA.cmake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio)
message(STATUS "opencv version ${OpenCV_VERSION}")

###########

include_directories(
  include
  copley_motion_library/inc
  copley_motion_library/inc/can
  ${XIMEA_INCLUDE_DIR}
)

ament_auto_add_library(ximea_usb_ros_driver SHARED src/ximea_usb_ros_driver/Node.cpp)
target_link_libraries(ximea_usb_ros_driver ${OpenCV_LIBRARIES} ${XIMEA_LIBRARY})
rclcpp_components_register_nodes(ximea_usb_ros_driver "${PROJECT_NAME}::Node")

ament_auto_add_executable(ximea_usb_ros_driver_node src/main.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)