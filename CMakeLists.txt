cmake_minimum_required(VERSION 2.8.3)

# 3rd-party Dependencies
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)


if(USE_ROSBUILD)
  message("USING ROSBUILD")
  # Use rosbuild
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  include_directories(${GSTREAMER_INCLUDE_DIRS})
  rosbuild_add_executable(gscam src/gscam.cpp)
  target_link_libraries(gscam ${GSTREAMER_LIBRARIES})
else()
  message("USING CATKIN")
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

  include_directories(${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
  add_executable(gscam src/gscam.cpp)
  target_link_libraries(gscam
    ${catkin_LIBRARIES}
    ${GSTREAMER_LIBRARIES})

  install(TARGETS gscam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

endif()

