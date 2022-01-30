#include <chrono>
#include <sensor_msgs/image_encodings.hpp>

#include "gscam/gscam.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

// Launch a test GStreamer pipeline and verify that at least 1 image was published
// See GSCAM_CONFIG in CMakeLists.txt
TEST(SmokeTest, smoke_test)  // NOLINT
{
  const size_t max_loops = 200;
  const std::chrono::milliseconds sleep_per_loop = std::chrono::milliseconds(10);
  const char * topic = "camera/image_raw";

  rclcpp::NodeOptions options{};
  // Future-proof: enable zero-copy IPC when it is available
  // https://github.com/ros-perception/image_common/issues/212
  // options.use_intra_process_comms(true);
  auto cam_node = std::make_shared<gscam::GSCam>(options);
  auto sub_node = std::make_shared<rclcpp::Node>("sub_node", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(cam_node);
  executor.add_node(sub_node);

  int count = 0;

  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    topic, 10,
    [&](const sensor_msgs::msg::Image::ConstSharedPtr image)  // NOLINT
    {
      // Match GSCAM_CONFIG and param defaults
      EXPECT_EQ(image->header.frame_id, "/camera_frame");
      EXPECT_EQ(image->width, 800u);
      EXPECT_EQ(image->height, 600u);
      EXPECT_EQ(image->encoding, sensor_msgs::image_encodings::RGB8);
      EXPECT_EQ(image->data.size(), image->width * image->height * 3);
      count++;
    }
  );

  test_rclcpp::wait_for_subscriber(sub_node, topic);

  EXPECT_EQ(0, count);

  for (size_t loop = 0; count < 1 && loop < max_loops; ++loop) {
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_some();
  }

  EXPECT_GT(count, 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
