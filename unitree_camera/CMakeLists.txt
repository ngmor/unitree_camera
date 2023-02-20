cmake_minimum_required(VERSION 3.8)
project(unitree_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(UnitreeCameraSDK REQUIRED)
find_package(PCL 1.12 REQUIRED QUIET) # bad solution
                                      # https://github.com/PointCloudLibrary/pcl/issues/2651
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(img_publisher src/img_publisher.cpp)
target_link_libraries(img_publisher UnitreeCameraSDK::UnitreeCameraSDK)
ament_target_dependencies(img_publisher
  rclcpp std_msgs sensor_msgs cv_bridge pcl_conversions Boost
)
add_executable(img_subscriber src/img_subscriber.cpp)

ament_target_dependencies(img_subscriber
  rclcpp std_msgs sensor_msgs cv_bridge
)
target_link_libraries(img_subscriber ${OpenCV_LIBRARIES})

install(TARGETS
  img_publisher
  img_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
