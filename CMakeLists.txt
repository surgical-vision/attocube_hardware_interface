## Core library
#add_library(${PROJECT_NAME} src/attocube_actors.cpp src/attocube_device_manager.cpp)
#set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "lib")
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}
#        ecc
#        ${catkin_LIBRARIES}
#        )
#
### Declare a C++ executable
### With catkin_make all packages are built within a single CMake context
### The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_example src/example.cpp src/attocube_actors.cpp)
#set_target_properties(${PROJECT_NAME}_example PROPERTIES OUTPUT_NAME example PREFIX "")
#add_dependencies(${PROJECT_NAME}_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_example
#     ${catkin_LIBRARIES}
#         ecc
# )
#
#add_executable(${PROJECT_NAME}_hardware_interface src/attocube_hardware_interface.cpp src/attocube_actors.cpp src/attocube_device_manager.cpp)
#set_target_properties(${PROJECT_NAME}_hardware_interface PROPERTIES OUTPUT_NAME hardware_interface PREFIX "")
#add_dependencies(${PROJECT_NAME}_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_hardware_interface
#        ${catkin_LIBRARIES}
#        ecc
#        )
#

cmake_minimum_required(VERSION 3.5)
project(attocube_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager REQUIRED)

include_directories(
        include
        #${controller_manager_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        SHARED
        src/attocube_actors.cpp
        src/attocube_device_manager.cpp
        src/attocube_hardware_interface.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ecc
        )

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        hardware_interface
        pluginlib
        rclcpp
        std_srvs
        )

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


# Utilities
add_executable(device_identifier
        src/utils/device_identifier.cpp
        )
target_link_libraries(device_identifier
        ${PROJECT_NAME}
        )

ament_target_dependencies(device_identifier
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        rclcpp
        std_srvs
        )

add_executable(actor_limits_cycle
        src/utils/actor_limits_cycle.cpp
        )
target_link_libraries(actor_limits_cycle
        ${PROJECT_NAME}
        )

ament_target_dependencies(actor_limits_cycle
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        rclcpp
        std_srvs
        )

# Installation
install(
        TARGETS
          ${PROJECT_NAME}
        DESTINATION lib
)
# Installation
install(
        TARGETS
          device_identifier
          actor_limits_cycle
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
        IiwaFriDriver
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()