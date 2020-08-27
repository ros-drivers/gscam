
#include <rclcpp/rclcpp.hpp>
#include <gscam/gscam.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  auto gscam_driver = std::make_shared<gscam::GSCam>(options);
  rclcpp::spin(gscam_driver);

  rclcpp::shutdown();
  return 0;
}

