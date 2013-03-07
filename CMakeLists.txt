cmake_minimum_required(VERSION 2.8.3)

# 3rd-party Dependencies
find_package(PkgConfig)
pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(OPENCV2 REQUIRED opencv2)


if(USE_ROSBUILD)
  # Use rosbuild
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  include_directories(${GLIB_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
  rosbuild_add_executable(gscam src/gscam.cpp)
  target_link_libraries(gscam
    gstreamer-0.10 gstbase-0.10 
    gobject-2.0 gmodule-2.0 gthread-2.0
    rt xml2 glib-2.0 gstapp-0.10
    pthread)
else()
  # Use Catkin
  project(gscam)

  find_package(catkin REQUIRED 
    COMPONENTS roscpp image_transport sensor_msgs camera_calibration_parsers)

  catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp image_transport sensor_msgs camera_calibration_parsers
    DEPENDS opencv2
    )

  include_directories(${catkin_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
  add_executable(gscam src/gscam.cpp)
  target_link_libraries(gscam
    ${catkin_LIBRARIES}
    gstreamer-0.10 gstbase-0.10 
    gobject-2.0 gmodule-2.0 gthread-2.0
    rt xml2 glib-2.0 gstapp-0.10
    pthread)

  install(TARGETS gscam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  

endif()

