cmake_minimum_required(VERSION 3.5)
project(imu_bno055)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-std=c++11 -li2c)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)


add_executable(bno055_i2c_node src/bno055_i2c_activity.cpp
  src/bno055_i2c_node.cpp src/watchdog.cpp)


ament_target_dependencies(bno055_i2c_node
        rclcpp
        sensor_msgs
        diagnostic_msgs
        std_srvs
        )
set(INCLUDE_DIRS ${ament_cmake_INCLUDE_DIRS} ${diagnostic_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} include/)
include_directories(${INCLUDE_DIRS})
set(LIBRARY_DIRS ${ament_cmake_LIBRARIES} ${diagnostic_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
target_link_libraries(bno055_i2c_node ${LIBRARY_DIRS})
set(LIBS ${ament_cmake_LIBRARIES} ${diagnostic_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})
#add_dependencies(bno055_i2c_node diagnostic_msgs_gencpp)



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

ament_export_dependencies(ament_cmake)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
