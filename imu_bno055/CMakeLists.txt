cmake_minimum_required(VERSION 3.5)
project(imu_bno055)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-li2c)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include src)

add_executable(bno055_i2c_node src/bno055_i2c_activity.cpp
  src/bno055_i2c_node.cpp src/watchdog.cpp)


ament_target_dependencies(bno055_i2c_node
        rclcpp
        sensor_msgs
        diagnostic_msgs
        std_srvs
        )


install(TARGETS bno055_i2c_node
        ARCHIVE
        DESTINATION lib
        LIBRARY
        DESTINATION lib
        RUNTIME
        DESTINATION bin)

install(TARGETS
        bno055_i2c_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(FILES
        config/default_muppet_imu_bno055_params.yaml
        DESTINATION share/${PROJECT_NAME}
        )

install(
        DIRECTORY include/
        DESTINATION include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)

ament_package()
