cmake_minimum_required(VERSION 3.8)
project(hole_toolpath_planner LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Hole.msg"
  "msg/HoleArray.msg"
  "msg/Toolpath.msg"
)

set(srv_files
  "srv/DetectHoles.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_core
  src/hole_detector.cpp
  src/parameters.cpp
  src/toolpath_generator.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_core
  PUBLIC ${PCL_DEFINITIONS}
)

ament_target_dependencies(${PROJECT_NAME}_core
  PUBLIC
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    pcl_conversions
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    ${cpp_typesupport_target})

add_library(${PROJECT_NAME}_component SHARED
  src/hole_detector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  PUBLIC
    ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_component
  PUBLIC
    rclcpp
    rclcpp_components
)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "hole_toolpath_planner::HoleToolpathPlannerNode")

add_executable(hole_detector_node src/hole_detector_node.cpp)

target_link_libraries(hole_detector_node
  PRIVATE
    ${PROJECT_NAME}_core
)

ament_target_dependencies(hole_detector_node
  PUBLIC
    rclcpp
    rclcpp_components
)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_component
  hole_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/resource
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

ament_package()
