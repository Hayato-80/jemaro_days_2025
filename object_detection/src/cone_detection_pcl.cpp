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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

// #include <my_msgs/msg/my_output_msg.hpp>


using namespace std::chrono_literals;

namespace rclcpp
{

class DetectionNode : public rclcpp::Node
{
public:
    DetectionNode(rclcpp::NodeOptions options) : Node("detection_node", options)
    {

        // pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      	// 	"/laser_scan/filtered_cleaned", 10, std::bind(&DetectionNode::pointcloud_callback, this, std::placeholders::_1));
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
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // Perform clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5); // Distance threshold
        ec.setMinClusterSize(50);    // Minimum points per cluster
        ec.setMaxClusterSize(5000);  // Maximum points per cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }

            // Calculate cluster centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);

            // Apply height and size constraints
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            float height = max_pt.z - min_pt.z;
            float base_diameter = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));

            if (height > 0.3 && height < 1.0 && base_diameter > 0.2 && base_diameter < 0.5) {
                // Publish marker
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = this->get_clock()->now();
                marker.header.frame_id = "map";
                marker.ns = "cone_detection";
                marker.id = 0;
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = centroid[0];
                marker.pose.position.y = centroid[1];
                marker.pose.position.z = centroid[2];
                marker.scale.x = base_diameter;
                marker.scale.y = base_diameter;
                marker.scale.z = height;
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker_publisher_->publish(marker);

                // Publish static TF
                geometry_msgs::msg::TransformStamped st;
                st.header.stamp = this->get_clock()->now();
                st.header.frame_id = "map";
                st.child_frame_id = "cone_1";
                st.transform.translation.x = centroid[0];
                st.transform.translation.y = centroid[1];
                st.transform.translation.z = centroid[2];
                st.transform.rotation.x = 0.0;
                st.transform.rotation.y = 0.0;
                st.transform.rotation.z = 0.0;
                st.transform.rotation.w = 1.0;
                tf_static_broadcaster_->sendTransform(st);
            }
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