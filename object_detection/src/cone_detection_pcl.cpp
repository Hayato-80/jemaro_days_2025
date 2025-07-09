#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp> // For iterating through PointCloud2 data
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp" // For PointStamped
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2_ros/buffer.h"                // For tf2_ros::Buffer
#include "tf2_ros/transform_listener.h"    // For tf2_ros::TransformListener
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"              // For TransformException
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For tf2::doTransform


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

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
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path_ref", 10, std::bind(&DetectionNode::path_callback, this, std::placeholders::_1));
        // init publishers
        // New publisher (in your constructor)
        colored_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/colored_pointcloud", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/filtered_cluster_poses", 10);
        
        marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        //        this->make_transforms(transformation);
        publishStaticTransform();
    }
  
private:
    // declare any subscriber / publisher / timer
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    // MyInputMsg input_msg;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pc_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    std::vector<std::pair<float, float>> path_waypoints_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "map"; // Parent frame
        static_transform.child_frame_id = "ZOE3/os_sensor"; // Child frame
        static_transform.transform.translation.x = 0.0; // Adjust as needed
        static_transform.transform.translation.y = 0.0; // Adjust as needed
        static_transform.transform.translation.z = 0.0; // Adjust as needed
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(static_transform);
    }

    std::vector<pcl::PointIndices> performClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(20);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        return cluster_indices;
    }

    // bool isConeDetected_intensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, Eigen::Vector4f& centroid, float& height, float& base_diameter, float& average_intensity) {
    //     pcl::compute3DCentroid(*cluster, centroid);

    //     pcl::PointXYZI min_pt, max_pt;
    //     pcl::getMinMax3D(*cluster, min_pt, max_pt);
    //     height = max_pt.z - min_pt.z;
    //     base_diameter = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));

    //     // Calculate average intensity
    //     float total_intensity = 0.0f;
    //     for (const auto& point : cluster->points) {
    //         total_intensity += point.intensity;
    //     }
    //     average_intensity = total_intensity / cluster->points.size();

    //     // Add intensity-based filtering criteria
    //     return (centroid[0] > 0.0 && std::abs(centroid[1]) < 2.0 &&
    //             height > 0.3 && height < 1.0 &&
    //             base_diameter > 0.2 && base_diameter < 0.5 &&
    //             average_intensity > 50.0); // Adjust intensity threshold as needed
    // }

    bool isConeDetected(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster, Eigen::Vector4f& centroid, float& height, float& base_diameter, float& average_intensity) {
        // Compute the centroid of the cluster
        pcl::compute3DCentroid(*cluster, centroid);

        // Compute the bounding box dimensions
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);
        height = max_pt.z - min_pt.z;
        base_diameter = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));

        // Calculate average intensity
        float total_intensity = 0.0f;
        for (const auto& point : cluster->points) {
            total_intensity += point.intensity;
        }
        average_intensity = total_intensity / cluster->points.size();

        // Debugging information
        RCLCPP_INFO(this->get_logger(), "Cluster centroid: (%f, %f, %f), height: %f, base diameter: %f, average intensity: %f",
                    centroid[0], centroid[1], centroid[2], height, base_diameter, average_intensity);

        // Check if the cluster resembles a cone or triangle shape
        float height_to_base_ratio = height / base_diameter;
        return (centroid[0] > 0.0 && std::abs(centroid[1]) < 5.0 && // Lateral range
                height > 0.5 && height < 1.0 && // Height range
                base_diameter > 0.1 && base_diameter < 1.0 && // Base diameter range
                height_to_base_ratio > 1.0 && height_to_base_ratio < 4.0 && // Cone-like ratio
                average_intensity >= -5.0); // Intensity threshold
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> filterConeClusters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    float distance_threshold)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cone_clusters;
        // // Variables to track the closest cluster in front of the car
        // pcl::PointCloud<pcl::PointXYZI>::Ptr closest_cluster = nullptr;
        // float min_distance = std::numeric_limits<float>::max();
        // float min_distance_to_path = std::numeric_limits<float>::max();


        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& index : indices.indices) {
                cluster->points.push_back(cloud->points[index]);
            }

            // Calculate bounding box dimensions
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            // float length = std::sqrt(std::pow(max_pt.x - min_pt.x, 2) + std::pow(max_pt.y - min_pt.y, 2));
            // float width = std::abs(max_pt.y - min_pt.y);
            float height = std::abs(max_pt.z - min_pt.z);

            // Filter out straight-line clusters
            // if (length > 3.0 * width && length > 3.0 * height) {
            //     RCLCPP_INFO(this->get_logger(), "Cluster removed: straight-line shape detected");
            //     continue; // Skip this cluster
            // }

            // Check if the cluster meets cone-specific criteria
            Eigen::Vector4f centroid;
            float base_diameter, average_intensity;
            if (isConeDetected(cluster, centroid, height, base_diameter, average_intensity)) {
                // Calculate distance to the closest waypoint in the path
                float min_distance = std::numeric_limits<float>::max();
                for (const auto& waypoint : path_waypoints_) {
                    float distance = std::sqrt(std::pow(centroid[0] - waypoint.first, 2) +
                                            std::pow(centroid[1] - waypoint.second, 2));
                    min_distance = std::min(min_distance, distance);
                }

                // Filter clusters based on distance threshold
                if (min_distance <= distance_threshold) {
                    cone_clusters.push_back(cluster);
                    RCLCPP_INFO(this->get_logger(), "Cluster added with distance to path: %f", min_distance);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Cluster removed: distance to path exceeds threshold (%f > %f)", min_distance, distance_threshold);
                }
            }
            
        }

        // // Add only the closest cluster to the cone clusters
        // if (closest_cluster) {
        //     cone_clusters.push_back(closest_cluster);
        //     RCLCPP_INFO(this->get_logger(), "Closest cluster added with distance to path: %f", min_distance_to_path);
        // }

        return cone_clusters;
    }

    void publishConeData(const Eigen::Vector4f& centroid, float height, float base_diameter, int cluster_id) {
        geometry_msgs::msg::PointStamped sensor_point, map_point;
        sensor_point.header.frame_id = "ZOE3/os_sensor";
        // sensor_point.header.stamp = this->get_clock()->now();
        sensor_point.header.stamp = rclcpp::Time(0); // Use the latest available transform
        sensor_point.point.x = centroid[0];
        sensor_point.point.y = centroid[1];
        sensor_point.point.z = centroid[2];

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
        marker.header.frame_id = "map";
        // marker.header.frame_id = "ZOE3/os_sensor"; // Use the correct frame ID
        marker.ns = "cluster_visualization";
        marker.id = cluster_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = centroid[0];
        marker.pose.position.y = centroid[1];
        marker.pose.position.z = centroid[2];
        marker.scale.x = 1.0; // Increase marker size
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_publisher_->publish(marker);
    }

    void publishColoredPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& filtered_clusters)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*cloud, *colored_cloud);

        float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
        int cluster_id = 0;

        for (const auto& cluster : filtered_clusters) {
            for (const auto& point : cluster->points) {
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = colors[cluster_id % 6][0];
                colored_point.g = colors[cluster_id % 6][1];
                colored_point.b = colors[cluster_id % 6][2];
                colored_cloud->points.push_back(colored_point);
            }
            cluster_id++;
        }

        colored_cloud->width = colored_cloud->points.size();
        colored_cloud->height = 1;
        colored_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*colored_cloud, output_msg);
        // output_msg.header.frame_id = "map"; // Publish in the global frame
        output_msg.header.frame_id = "ZOE3/os_sensor"; // Use the correct
        // output_msg.header.frame_id = "map"; // Use the correct frame ID
        output_msg.header.stamp = this->get_clock()->now();
        colored_pc_publisher_->publish(output_msg);
    }

    void removeFlatSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
    {
        // Step 1: Downsample the point cloud using Voxel Grid
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
        voxelGrid.setInputCloud(cloud);
        voxelGrid.setLeafSize(0.1, 0.1, 0.1); // Adjust leaf size as needed
        voxelGrid.filter(*cloud_filtered);

        // Step 2: Detect flat surfaces using SAC Segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(0.03); // Adjust threshold as needed
        seg.setDistanceThreshold(0.05); // Default
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        // Step 3: Remove inliers (flat surface points)
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true); // Keep points that are not part of the plane
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        extract.filter(*final_cloud);

        // Replace the original cloud with the filtered cloud
        cloud.swap(final_cloud);
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!path_waypoints_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Path already set. Ignoring new path updates.");
            return; // Ignore subsequent updates
        }

        for (const auto& pose : msg->poses) {
            path_waypoints_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        // Log the number of waypoints
        RCLCPP_INFO(this->get_logger(), "Path waypoints: %zu", path_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Static path set with %zu waypoints", path_waypoints_.size());
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *cloud);

        // Remove flat surfaces
        removeFlatSurfaces(cloud);

        // Perform clustering
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud(*cloud, *cloud_xyz);
        std::vector<pcl::PointIndices> cluster_indices = performClustering(cloud_xyz);

        // Filter cone clusters
        float distance_threshold = 5000.0;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cone_clusters = filterConeClusters(cloud_xyz, cluster_indices, distance_threshold);
        RCLCPP_INFO(this->get_logger(), "Detected %zu clusters", cluster_indices.size());
        RCLCPP_INFO(this->get_logger(), "Filtered %zu cone clusters", cone_clusters.size());

        publishColoredPointCloud(cloud_xyz, cone_clusters);

        // Publish poses of filtered clusters
        int cluster_id = 0;
        for (const auto& cluster : cone_clusters) {
            Eigen::Vector4f centroid;
            float height, base_diameter, average_intensity;
            if (isConeDetected(cluster, centroid, height, base_diameter, average_intensity)) {
                geometry_msgs::msg::PointStamped sensor_point, map_point;
                sensor_point.header.frame_id = "ZOE3/os_sensor"; // Sensor frame
                // sensor_point.header.stamp = this->get_clock()->now();
                sensor_point.header.stamp = rclcpp::Time(0); // Use the latest available transform
                sensor_point.point.x = centroid[0];
                sensor_point.point.y = centroid[1];
                sensor_point.point.z = centroid[2];

                try {
                    if (tf_buffer_->canTransform("map", "ZOE3/os_sensor", tf2::TimePointZero)) {
                        map_point = tf_buffer_->transform(sensor_point, "map", tf2::durationFromSec(1.0));

                        // Publish the transformed pose
                        geometry_msgs::msg::PoseStamped pose_msg;
                        pose_msg.header.frame_id = "map"; // Publish in the map frame
                        pose_msg.header.stamp = this->get_clock()->now();
                        pose_msg.pose.position.x = map_point.point.x;
                        pose_msg.pose.position.y = map_point.point.y;
                        pose_msg.pose.position.z = map_point.point.z;
                        pose_msg.pose.orientation.x = 0.0;
                        pose_msg.pose.orientation.y = 0.0;
                        pose_msg.pose.orientation.z = 0.0;
                        pose_msg.pose.orientation.w = 1.0;

                        RCLCPP_INFO(this->get_logger(), "Publishing transformed pose for cluster %d at (%f, %f, %f)", cluster_id, map_point.point.x, map_point.point.y, map_point.point.z);
                        pose_publisher_->publish(pose_msg);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Transform not available from [ZOE3/os_sensor] to [map]");
                    }
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                }
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