cmake_minimum_required(VERSION 2.8.3)

project(gscam)

# System Dependencies
find_package(PkgConfig)

pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()

if(USE_ROSBUILD)
  # Use rosbuild
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  include_directories(${GLIB_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

  rosbuild_add_library(gscam src/gscam.cpp)
  target_link_libraries(gscam
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})

  rosbuild_add_executable(gscam_node src/gscam_node.cpp)
  target_link_libraries(gscam_node gscam
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})
  set_target_properties(gscam_node PROPERTIES OUTPUT_NAME gscam)

  rosbuild_add_library(GSCamNodelet src/gscam_nodelet.cpp)
  target_link_libraries(GSCamNodelet gscam
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})

else()
  # Use Catkin
  find_package(catkin REQUIRED 
    COMPONENTS roscpp image_transport sensor_msgs nodelet
    camera_calibration_parsers camera_info_manager
    )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gscam
    CATKIN_DEPENDS roscpp nodelet image_transport sensor_msgs
    camera_calibration_parsers camera_info_manager
    DEPENDS GSTREAMER GST_APP
    )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GLIB_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS})

  add_library(gscam src/gscam.cpp)
  target_link_libraries(gscam
    ${catkin_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})

  add_executable(gscam_node src/gscam_node.cpp)
  target_link_libraries(gscam_node gscam
    ${catkin_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})
  set_target_properties(gscam_node PROPERTIES OUTPUT_NAME gscam)

  add_library(GSCamNodelet src/gscam_nodelet.cpp)
  target_link_libraries(GSCamNodelet gscam
    ${catkin_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES})

  # Install directives

  install(TARGETS gscam gscam_node GSCamNodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)

  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  install(FILES
    examples/v4l.launch
    examples/gscam_nodelet.launch
    examples/nodelet_pipeline.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  install(FILES examples/uncalibrated_parameters.ini
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
    )

  # Interim compatibility
  # Remove this in the next release
  install(FILES ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/gscam_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(EXECUTABLE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
endif()

# Interim compatibility
# Remove this in the next distribution release
configure_file(scripts/gscam_node.in ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/gscam_node)
file(COPY ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/gscam_node 
  DESTINATION ${EXECUTABLE_OUTPUT_PATH}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)


