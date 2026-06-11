cmake_minimum_required(VERSION 3.15)
project(picam1dof)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Cmd.msg" "srv/Detect.srv")
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")

ament_auto_add_library(process_image_node SHARED src/process_image.cpp
src/color_detector.cpp src/flow_detector.cpp src/flow_point.cpp src/face_detector.cpp)
target_include_directories(process_image_node PRIVATE include)
target_link_libraries(process_image_node "${cpp_typesupport_target}" ${OpenCV_LIBS})

# auto-executable from node
rclcpp_components_register_node(process_image_node
  PLUGIN "picam1dof::ProcessImage"
  EXECUTABLE process_image)

endif()

# install(TARGETS process_image_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
#   )
  
install(PROGRAMS src/pwm.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch haar
  DESTINATION share/${PROJECT_NAME})
  
# ament_export_libraries(process_image_node)
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
