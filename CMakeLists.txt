cmake_minimum_required(VERSION 3.17)
project(pcl_to_ply LANGUAGES C CXX)

# ros2 node executable
add_executable(${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_to_ply.cpp")
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# create ament package
find_package(ament_cmake REQUIRED)
list(APPEND ROS_PKGS rclcpp sensor_msgs pcl_ros)
foreach(PKG ${ROS_PKGS})
    find_package(${PKG} REQUIRED)
endforeach()
ament_target_dependencies(${PROJECT_NAME} ${ROS_PKGS})
ament_package()

# install to ros ws lib dir
install(TARGETS ${PROJECT_NAME} DESTINATION "lib/${PROJECT_NAME}")
install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
