// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GSCAM__GSCAM_HPP_
#define GSCAM__GSCAM_HPP_

#include <stdexcept>
#include <string>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"

namespace gscam
{

class GSCam : public rclcpp::Node
{
public:
  explicit GSCam(const rclcpp::NodeOptions & options);
  ~GSCam();

private:
  bool configure();
  bool init_stream();
  void publish_stream();
  void cleanup_stream();

  void run();

  // General gstreamer configuration
  std::string gsconfig_;

  // Gstreamer structures
  GstElement * pipeline_;
  GstElement * sink_;

  // Appsink configuration
  bool sync_sink_;
  bool preroll_;
  bool reopen_on_eof_;
  bool use_gst_timestamps_;

  // Camera publisher configuration
  std::string frame_id_;
  int width_, height_;
  std::string image_encoding_;
  std::string camera_name_;
  std::string camera_info_url_;
  bool use_sensor_data_qos_;

  // ROS Inteface
  // Calibration between ros::Time and gst timestamps
  uint64_t time_offset_;
  camera_info_manager::CameraInfoManager camera_info_manager_;
  image_transport::CameraPublisher camera_pub_;
  // Case of a jpeg only publisher
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;

  // Poll gstreamer on a separate thread
  std::thread pipeline_thread_;
  std::atomic<bool> stop_signal_;
};

}  // namespace gscam

#endif  // GSCAM__GSCAM_HPP_
