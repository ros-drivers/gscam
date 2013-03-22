#ifndef __GSCAM_GSCAM_H
#define __GSCAM_GSCAM_H

extern "C"{
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <stdexcept>

namespace gscam {

  class GSCam {
  public:
    GSCam(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~GSCam();

    bool configure();
    bool init_stream();
    void publish_stream();
    void cleanup_stream();

    void run();

  private:
    bool set_camera_info_cb(
        sensor_msgs::SetCameraInfo::Request &req,
        sensor_msgs::SetCameraInfo::Response &rsp);

    // General gstreamer configuration
    std::string gsconfig_;

    // Gstreamer structures
    GstElement *pipeline_;
    GstElement *sink_;

    // Appsink configuration
    bool sync_sink_;
    bool preroll_;
    bool reopen_on_eof_;

    // Camera publisher configuration
    std::string frame_id_;
    int width_, height_;
    sensor_msgs::CameraInfo camera_info_;
    std::string camera_name_;
    std::string camera_parameters_file_;

    // ROS Inteface
    ros::NodeHandle nh_, nh_private_;
    image_transport::ImageTransport image_transport_;
    image_transport::CameraPublisher camera_pub_;
    ros::ServiceServer set_camera_info_servivce_;
  };

}

#endif // ifndef __GSCAM_GSCAM_H
