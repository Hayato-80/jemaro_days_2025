#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>

#include <sensor_msgs/point_cloud2_iterator.hpp> // For iterating through PointCloud2 data
#include <cmath>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"                // For tf2_ros::Buffer
#include "tf2_ros/transform_listener.h"    // For tf2_ros::TransformListener
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "geometry_msgs/msg/point_stamped.hpp" // For PointStamped
#include "tf2/exceptions.h"              // For TransformException

#include "tf2_ros/transform_broadcaster.h"

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

        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      		"/laser_scan/filtered_cleaned", 10, std::bind(&DetectionNode::pointcloud_callback, this, std::placeholders::_1));
        // pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      	// 	"/ZOE3/os_node/points", 10, std::bind(&DetectionNode::pointcloud_callback, this, std::placeholders::_1));
        
        // init publishers
        // New publisher (in your constructor)
        colored_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_pointcloud", 10);

        marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        //        this->make_transforms(transformation);
    }
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    // MyInputMsg input_msg;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pc_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<pcl::PointIndices> performClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.2);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(15);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        return cluster_indices;
    }

    bool isConeDetected(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, Eigen::Vector4f& centroid, float& height, float& base_diameter) {
        pcl::compute3DCentroid(*cluster, centroid);

        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        height = max_pt.z - min_pt.z;
        base_diameter = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));

        return (centroid[0] > 0.0 && std::abs(centroid[1]) < 2.0 &&
                height > 0.3 && height < 1.0 &&
                base_diameter > 0.2 && base_diameter < 0.5);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filterConeClusters(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cone_clusters;

        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }

            // Calculate bounding box dimensions
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            float length = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));
            float width = std::abs(max_pt.y - min_pt.y);
            float height = std::abs(max_pt.z - min_pt.z);

            // Filter out straight-line clusters
            if (length > 5 * width && length > 5 * height) {
                RCLCPP_INFO(this->get_logger(), "Cluster removed: straight-line shape detected");
                continue; // Skip this cluster
            }

            // Check if the cluster meets cone-specific criteria
            Eigen::Vector4f centroid;
            float base_diameter;
            if (isConeDetected(cluster, centroid, height, base_diameter)) {
                cone_clusters.push_back(cluster);
            }
        }

        return cone_clusters;
    }

    void publishConeData(const Eigen::Vector4f& centroid, float height, float base_diameter, int cluster_id) {
        geometry_msgs::msg::PointStamped sensor_point, map_point;
        sensor_point.header.frame_id = "ZOE3/os_sensor";
        sensor_point.header.stamp = this->get_clock()->now();
        sensor_point.point.x = centroid[0];
        sensor_point.point.y = centroid[1];
        sensor_point.point.z = centroid[2];

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "ZOE3/os_sensor";
        marker.ns = "cone_detection";
        marker.id = cluster_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = sensor_point.point.x;
        marker.pose.position.y = sensor_point.point.y;
        marker.pose.position.z = sensor_point.point.z;
        marker.scale.x = base_diameter;
        marker.scale.y = base_diameter;
        marker.scale.z = height;
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_publisher_->publish(marker);

        geometry_msgs::msg::TransformStamped st;
        st.header.stamp = this->get_clock()->now();
        st.header.frame_id = "map";
        st.child_frame_id = "cones" + std::to_string(cluster_id);
        st.transform.translation.x = sensor_point.point.x;
        st.transform.translation.y = sensor_point.point.y;
        st.transform.translation.z = sensor_point.point.z;
        st.transform.rotation.x = 0.0;
        st.transform.rotation.y = 0.0;
        st.transform.rotation.z = 0.0;
        st.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(st);
    }

    void publishClusterMarkers(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, int cluster_id) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "ZOE3/os_sensor";
        marker.ns = "cluster_visualization";
        marker.id = cluster_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_publisher_->publish(marker);
    }

    void publishColoredPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::vector<pcl::PointIndices>& cluster_indices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *colored_cloud);

        float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
        int cluster_id = 0;

        for (const auto& indices : cluster_indices) {
            for (const auto& index : indices.indices) {
                colored_cloud->points[index].r = colors[cluster_id % 6][0];
                colored_cloud->points[index].g = colors[cluster_id % 6][1];
                colored_cloud->points[index].b = colors[cluster_id % 6][2];
            }
            cluster_id++;
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        output_msg.header.frame_id = "ZOE3/os_sensor";
        output_msg.header.stamp = this->get_clock()->now();
        colored_pc_publisher_->publish(output_msg);
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<pcl::PointIndices> cluster_indices = performClustering(cloud);
         std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cone_clusters = filterConeClusters(cloud, cluster_indices);
        RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", cluster_indices.size());

        publishColoredPointCloud(cloud, cluster_indices);
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