cmake_minimum_required(VERSION 3.8)
project(spectral_fubar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
find_package(gazebo_msgs REQUIRED)

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

include_directories( ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

## walker_lib can now be linked to any test_exec or runtime_exec possible
add_library(inspector_lib libs/HuskyInspector.cpp)
target_include_directories(inspector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(inspector_lib  rclcpp std_msgs geometry_msgs sensor_msgs gazebo_msgs)
ament_export_targets(inspector_lib HAS_LIBRARY_TARGET)
# install the include directory to the install/include/
install(
  DIRECTORY include
  DESTINATION include
)
install(
  TARGETS inspector_lib
  EXPORT inspector_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

## actual inspector node which links against inspector_lib
add_executable(inspector src/main.cpp)
target_include_directories(inspector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(inspector inspector_lib)
ament_target_dependencies(inspector rclcpp std_msgs geometry_msgs sensor_msgs gazebo_msgs)
install(TARGETS inspector
  DESTINATION lib/${PROJECT_NAME})




# # Install headers
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include
# )

install(
  DIRECTORY launch worlds models rviz urdf
  DESTINATION share/${PROJECT_NAME}
)



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
