cmake_minimum_required(VERSION 3.8)
project(spectral_fubar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Doxygen REQUIRED)

find_package(realsense2 2.54.1)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  ${PROJECT_SOURCE_DIR}/test
  )

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

install(
  DIRECTORY launch worlds models rviz
  DESTINATION share/${PROJECT_NAME}
)

include_directories(include)

set(node_plugins "")

# set(SOURCES
#     src/base_realsense_node.cpp
#     src/parameters.cpp
#     src/tfs.cpp
#     src/rs_node_setup.cpp
#     src/named_filter.cpp
#     src/profile_manager.cpp
#     src/ros_sensor.cpp
#     src/ros_utils.cpp
#     src/dynamic_params.cpp
#     src/image_publisher.cpp
#     src/ros_param_backend.cpp
#   )

# set(INCLUDES
#     include/${PROJECT_NAME}/ros_sensor.h
#     include/${PROJECT_NAME}/dynamic_params.h
#     include/${PROJECT_NAME}/sensor_params.h
#     include/${PROJECT_NAME}/named_filter.h
#     include/${PROJECT_NAME}/ros_param_backend.h
#     include/${PROJECT_NAME}/image_publisher.h
#     include/${PROJECT_NAME}/constants.h
#     include/${PROJECT_NAME}/ros_utils.h
#     include/${PROJECT_NAME}/base_realsense_node.h
#     include/${PROJECT_NAME}/image_publisher.h)

# add_library(${PROJECT_NAME} SHARED
#     ${INCLUDES}
#     ${SOURCES}
# )

# target_link_libraries(${PROJECT_NAME}
#     ${realsense2_LIBRARY}
# )

# set(dependencies
#   cv_bridge
#   image_transport
#   rclcpp
#   rclcpp_components
#   realsense2_camera_msgs
#   std_msgs
#   sensor_msgs
#   nav_msgs
#   tf2
#   realsense2
#   tf2_ros
#   diagnostic_updater
# )

# ament_target_dependencies(${PROJECT_NAME}
#   ${dependencies}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_unit_test test/unit_test.cpp)
  target_include_directories(${PROJECT_NAME}_unit_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )


  ament_add_gtest(${PROJECT_NAME}_integration_test test/integration_test.cpp)
  target_include_directories(${PROJECT_NAME}_integration_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_integration_test rclcpp  rclcpp_components sensor_msgs std_msgs)
endif()

# Ament exports
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(${dependencies})

ament_package()
