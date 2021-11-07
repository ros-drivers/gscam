cmake_minimum_required(VERSION 3.5)

project(gscam)

# System Dependencies
find_package(PkgConfig)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
  include
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS})

add_library(gscam SHARED src/gscam.cpp)
target_link_libraries(gscam
  ${ament_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES})
ament_target_dependencies(gscam
  "class_loader"
  "image_transport"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "camera_calibration_parsers"
  "camera_info_manager")
rclcpp_components_register_nodes(gscam "gscam::GSCam")
set(node_plugins "${node_plugins}gscam::GSCam;$<TARGET_FILE:gscam>\n")

add_executable(gscam_node src/gscam_node.cpp)
target_link_libraries(gscam_node gscam
  ${ament_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES})
set_target_properties(gscam_node PROPERTIES OUTPUT_NAME gscam)

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_gscam)

ament_export_libraries(gscam)

ament_package()

# Install directives

install(TARGETS gscam
  EXPORT export_gscam
  DESTINATION lib
  )

install(TARGETS gscam_node
  DESTINATION lib/gscam
  )

install(FILES
  examples/component_pipeline_launch.py
  examples/v4l.launch
  examples/v4ljpeg.launch
  examples/videofile.launch
  DESTINATION share/gscam/launch
  )

install(FILES examples/uncalibrated_parameters.ini
  DESTINATION share/gscam/examples
  )
