cmake_minimum_required(VERSION 3.5)
project(picam1dof)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Cmd.msg" "srv/Detect.srv")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(process_image_node SHARED src/process_image.cpp
src/color_detector.cpp src/flow_detector.cpp src/flow_point.cpp src/face_detector.cpp)
target_include_directories(process_image_node PRIVATE 
                           include
                           ${OpenCV_INCLUDE_DIRS}
                           ${YamlCpp_INCLUDE_DIRS})
ament_target_dependencies(
  process_image_node rclcpp cv_bridge image_transport rclcpp_components)
target_link_libraries(process_image_node "${cpp_typesupport_target}" ${OpenCV_LIBRARIES})

# auto-executable from node
rclcpp_components_register_node(process_image_node
  PLUGIN "picam1dof::ProcessImage"
  EXECUTABLE process_image)

install(TARGETS process_image_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  
install(PROGRAMS src/pwm.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch haar
  DESTINATION share/${PROJECT_NAME})
  
ament_export_libraries(process_image_node)
ament_package()
