cmake_minimum_required(VERSION 3.8)
project(vid_publisher)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher
  rclcpp
  std_msgs
)
install(TARGETS publisher
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(vid_pub src/vid_pub.cpp)
ament_target_dependencies(vid_pub
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
)
install(TARGETS vid_pub
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(vid_pub_2 src/vid_pub_2.cpp)
ament_target_dependencies(vid_pub_2
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
)
install(TARGETS vid_pub_2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(imshow_test src/imshow_test.cpp)
ament_target_dependencies(imshow_test
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  cv_bridge
)
install(TARGETS imshow_test
  DESTINATION lib/${PROJECT_NAME}
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
