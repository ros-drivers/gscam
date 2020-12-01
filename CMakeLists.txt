cmake_minimum_required(VERSION 3.0.2)

project(gscam)

# System Dependencies
# pkg-config libraries
find_package(PkgConfig REQUIRED)

#
# For Noetic 
#   - removed ROSBUILD target (deprecated)
#   - should remove gstreamer-0.10 support (old)
#

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


# Use Catkin build tools.
find_package(catkin REQUIRED COMPONENTS roscpp image_transport sensor_msgs camera_calibration_parsers camera_info_manager )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp image_transport sensor_msgs camera_calibration_parsers camera_info_manager
  DEPENDS GSTREAMER GST_APP
)

# Build defs
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gscam.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/gscam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

# Install directives

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(FILES examples/uncalibrated_parameters.ini
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

set(EXECUTABLE_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})



