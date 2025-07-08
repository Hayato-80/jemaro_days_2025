
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

#include <sensor_msgs/point_cloud2_iterator.hpp> // For iterating through PointCloud2 data
#include <cmath>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"

// #include <my_msgs/msg/my_output_msg.hpp>


using namespace std::chrono_literals;

namespace rclcpp
{

class DetectionNode : public rclcpp::Node
{
public:
    DetectionNode(rclcpp::NodeOptions options) : Node("detection_node", options)
    {
        // init subscribers
        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      		"/ZOE3/os_node/points", 10, std::bind(&DetectionNode::pointcloud_callback, this, std::placeholders::_1));
            
        // init publishers
        marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        // tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        //        this->make_transforms(transformation);
    }
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    // MyInputMsg input_msg;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
      sensor_msgs::msg::PointCloud2 pc_msg = *msg;
      sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_intensity(pc_msg, "intensity");

      float cone_x = 0.0, cone_y = 0.0, cone_z = 0.0;
      float intensity_threshold = 200.0; // Example threshold for intensity
      int point_count = 0;

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
          if (*iter_intensity > intensity_threshold) {
              cone_x += *iter_x;
              cone_y += *iter_y;
              cone_z += *iter_z;
              point_count++;
          }
      }

      if (point_count > 0) {
          // Calculate the centroid of the detected cone
          cone_x /= point_count;
          cone_y /= point_count;
          cone_z /= point_count;

          // Publish the marker
          visualization_msgs::msg::Marker marker;
          marker.header.stamp = this->get_clock()->now();
          marker.header.frame_id = "map";
          marker.ns = "cone_detection";
          marker.id = 0;
          marker.type = visualization_msgs::msg::Marker::SPHERE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.position.x = cone_x;
          marker.pose.position.y = cone_y;
          marker.pose.position.z = cone_z;
          marker.scale.x = 0.5;  // Size of the marker
          marker.scale.y = 0.5;
          marker.scale.z = 0.5;
          marker.color.r = 1.0;  // Red color
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;  // Fully opaque
          marker_publisher_->publish(marker);

          // Publish the static TF
          geometry_msgs::msg::TransformStamped st;
          st.header.stamp = this->get_clock()->now();
          st.header.frame_id = "map";
          st.child_frame_id = "cone_1";
          st.transform.translation.x = cone_x;
          st.transform.translation.y = cone_y;
          st.transform.translation.z = cone_z;
          st.transform.rotation.x = 0.0;
          st.transform.rotation.y = 0.0;
          st.transform.rotation.z = 0.0;
          st.transform.rotation.w = 1.0;
          tf_static_broadcaster_->sendTransform(st);
      }

    }
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<rclcpp::DetectionNode>(options));
  rclcpp::shutdown();
  return 0;
}