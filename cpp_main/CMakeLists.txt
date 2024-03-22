cmake_minimum_required(VERSION 3.8)
project(cpp_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add source files
set(SOURCES
    src/utils.cpp
    src/functions.cpp
    src/project.cpp
    include/utils.h
    include/functions.h
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# PCL
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# path planner
add_executable(planner ${SOURCES})
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(planner rclcpp sensor_msgs nav_msgs geometry_msgs realsense2 OpenCV)
# ament_target_dependencies(planner rclcpp sensor_msgs nav_msgs geometry_msgs tf2_msgs realsense2 OpenCV)
target_link_libraries(planner ${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
# target_link_libraries(planner ${OpenCV_LIBS})

# GPS node
# add_executable(gps_sub src/gps_node.cpp)
# ament_target_dependencies(gps_sub rclcpp sensor_msgs)


# MQTT
# add_library(paho-mqtt3as src/mqtt_client.cpp)
# add_library(paho-mqttpp3 src/mqtt_client.cpp)

# find_library(paho-mqtt3a NAMES libpaho-mqtt3a.so REQUIRED)
# find_library(paho-mqttpp3 NAMES libpaho-mqttpp3.a REQUIRED)
# add_library(pahottpp STATIC IMPORTED)
# set_target_properties(pahottpp PROPERTIES IMPORTED_LOCATION ${paho-mqttpp3})
# add_library(pahomqtt3a SHARED IMPORTED)
# set_property(TARGET pahomqtt3a PROPERTY IMPORTED_LOCATION ${paho-mqtt3a})
# add_executable(mqtt_client src/mqtt_client.cpp)
# target_link_libraries(mqtt_client pahottpp)
# target_link_libraries(mqtt_client pahomqtt3a)

install(TARGETS
  planner
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#   planner
#   gps_sub
#   mqtt_client
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
