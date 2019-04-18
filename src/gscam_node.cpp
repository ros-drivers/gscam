
#include <ros/ros.h>
#include <gscam/gscam.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "gscam_publisher");
  ros::NodeHandle nh, nh_private("~");

  ROS_INFO("Testing node!!!!");

  gscam::GSCam gscam_driver(nh, nh_private);
  // gscam_driver.run();
  boost::thread gscam_thread = boost::thread(boost::bind(&gscam::GSCam::run, &gscam_driver));

  ros::spin();

  gscam_thread.join();

  return 0;
}
