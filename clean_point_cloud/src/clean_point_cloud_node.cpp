#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>

class PointCloudFilterNode : public rclcpp::Node
{
public:
  PointCloudFilterNode()
      : Node("clean_point_cloud_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ZOE3/os_node/points_downsampled",
        rclcpp::SensorDataQoS(), // Fix QoS mismatch with LIDAR
        std::bind(&PointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_scan/filtered_cleaned", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto filtered_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Copy metadata
    *filtered_msg = *msg;
    filtered_msg->data.clear();

    size_t point_size = msg->point_step;
    size_t num_points = msg->width * msg->height;

    for (size_t i = 0; i < num_points; ++i)
    {
      const uint8_t *point_ptr = &msg->data[i * point_size];

      // Extract X, Y and Z
      float x = *reinterpret_cast<const float *>(point_ptr + 0); // x at offset 0
      float y = *reinterpret_cast<const float *>(point_ptr + 4); // y at offset 4
      float z = *reinterpret_cast<const float *>(point_ptr + 8); // z at offset 8

      if ((z >= -3.0f && z <= 0.0f) && (y <= 3.0f) && (y >= -1.5f) && (x < 25.0f) && (x > 0.0f))
      {
        // Keep point
        filtered_msg->data.insert(filtered_msg->data.end(), point_ptr, point_ptr + point_size);
      }
    }

    filtered_msg->width = filtered_msg->data.size() / point_size;
    filtered_msg->height = 1;
    filtered_msg->row_step = filtered_msg->width * point_size;
    filtered_msg->is_dense = false;

    publisher_->publish(*filtered_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}
