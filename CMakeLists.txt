cmake_minimum_required(VERSION 3.8)
project(trapezoidal_joint_controller)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PACKAGE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

############################ Dependencies ##################################
set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    hardware_interface
    pluginlib
    std_msgs
    control_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    trajectory_msgs
    generate_parameter_library
    sensor_msgs
    controller_utils
    trapezoidal_motion_planner
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  trapezoidal_joint_controller_parameters
  src/trapezoidal_joint_controller_parameters.yaml
)

############################ Library ##################################
add_library(${PROJECT_NAME}
  SHARED
  src/CommandProxy.cpp
  src/read_write_interfaces/ReferenceInterfaces.cpp
  src/TrapezoidalJointController.cpp
  ${GENERATED_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
 trapezoidal_joint_controller_parameters
)

############################# Plugin Export ##################################
target_compile_definitions(${PROJECT_NAME}
 PRIVATE 
 "${PROJECT_NAME_UPPER}_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(controller_interface trapezoidal_joint_controller.xml)

############################# Installation ##################################
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  trapezoidal_joint_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(export_trapezoidal_joint_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
