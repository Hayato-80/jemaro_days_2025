#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

struct Obstacle
{
  double x;
  double y;
  bool passed = false;
};

class LaneSwitchingPlanner : public rclcpp::Node
{
public:
  LaneSwitchingPlanner() : Node("lane_switching_planner"), current_state_(State::REFERENCE_PATH)
  {
    // Declare parameters with default values
    this->declare_parameter<double>("trigger_distance", 20.0);
    this->declare_parameter<double>("avoidance_duration", 1.5);
    this->declare_parameter<double>("transition_duration", 4.0);
    this->declare_parameter<double>("return_transition_duration", 3.0);
    this->declare_parameter<double>("left_lane_position_ratio", 0.3);
    this->declare_parameter<double>("path_extension_length", 50.0);
    this->declare_parameter<double>("path_extension_resolution", 1.0);
    this->declare_parameter<double>("obstacle_min_separation", 2.0); // Minimum distance between obstacles

    // Get parameter values
    trigger_distance_ = this->get_parameter("trigger_distance").as_double();
    avoidance_duration_ = this->get_parameter("avoidance_duration").as_double();
    transition_duration_ = this->get_parameter("transition_duration").as_double();
    return_transition_duration_ = this->get_parameter("return_transition_duration").as_double();
    left_lane_position_ratio_ = this->get_parameter("left_lane_position_ratio").as_double();
    path_extension_length_ = this->get_parameter("path_extension_length").as_double();
    path_extension_resolution_ = this->get_parameter("path_extension_resolution").as_double();
    obstacle_min_separation_ = this->get_parameter("obstacle_min_separation").as_double();

    // Create subscribers
    right_threshold_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path_right", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
          right_threshold_ = msg;
        });

    left_threshold_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path_left", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
          left_threshold_ = msg;
        });

    ref_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path_ref", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
          ref_path_ = msg;
          if (current_state_ == State::REFERENCE_PATH || current_state_ == State::TRANSITION_TO_REF)
          {
            reference_path_ = *msg;
          }
        });

    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ZOE3/position/map_ekf_odometry", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        {
          current_pose_ = msg->pose.pose;
          checkObstacleProximity();
        });

    // New subscriber for obstacle positions
    obstacles_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cone_poses", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          updateObstaclePositions(msg->pose.position.x, msg->pose.position.y);
        });

    // Create publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ZOE3/path_follower/setPath", 10);

    // Timer for continuous path publishing and state management
    publish_timer_ = this->create_wall_timer(
        500ms, [this]()
        {
          updatePath();
          nav_msgs::msg::Path extended_path = extendPath(reference_path_);
          path_pub_->publish(extended_path); });

    return_check_timer_ = this->create_wall_timer(
        50ms, [this]()
        {
          if (current_state_ == State::LEFT_LANE) {
            auto elapsed = (this->now() - transition_start_time_).seconds();
            if (elapsed >= avoidance_duration_) {
              current_state_ = State::TRANSITION_TO_REF;
              transition_start_time_ = this->now();
              obstacle_positions_.clear(); // <-- Clear all obstacles when returning
              RCLCPP_INFO(this->get_logger(), "Avoidance complete. Returning to reference path");
            }
          } });

    RCLCPP_INFO(this->get_logger(), "Lane switching planner initialized");
    RCLCPP_INFO(this->get_logger(), "Will switch lanes when obstacles are within %.1f meters", trigger_distance_);
    RCLCPP_INFO(this->get_logger(), "Left lane position: %.0f%% from left threshold", left_lane_position_ratio_ * 100);
  }

private:
  enum class State
  {
    REFERENCE_PATH,     // Driving in reference path (right lane)
    TRANSITION_TO_LEFT, // Moving to left lane
    LEFT_LANE,          // Driving in left lane (35% from left threshold)
    TRANSITION_TO_REF   // Returning to reference path
  };

  void updateObstaclePositions(double x, double y)
  {
    // Check if this position is too close to existing obstacles
    for (auto &obstacle : obstacle_positions_)
    {
      double dx = x - obstacle.x;
      double dy = y - obstacle.y;
      if (std::hypot(dx, dy) < obstacle_min_separation_)
      {
        return;
      }
    }

    // Add new obstacle position
    obstacle_positions_.push_back({x, y, false});

    // Limit the number of obstacles tracked
    if (obstacle_positions_.size() > 3)
    {
      obstacle_positions_.erase(obstacle_positions_.begin());
    }
  }

  nav_msgs::msg::Path extendPath(const nav_msgs::msg::Path &original_path)
  {
    nav_msgs::msg::Path extended_path = original_path;
    if (original_path.poses.empty())
      return extended_path;

    // Get the last pose in the original path
    auto last_pose = original_path.poses.back().pose;

    // Calculate the direction vector from the second-to-last to last pose
    geometry_msgs::msg::Vector3 direction;
    if (original_path.poses.size() > 1)
    {
      auto second_last = original_path.poses.end()[-2].pose;
      direction.x = last_pose.position.x - second_last.position.x;
      direction.y = last_pose.position.y - second_last.position.y;
      direction.z = last_pose.position.z - second_last.position.z;

      // Normalize the direction vector
      double length = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
      if (length > 0)
      {
        direction.x /= length;
        direction.y /= length;
        direction.z /= length;
      }
    }
    else
    {
      // If only one point, assume forward along x-axis
      direction.x = 1.0;
      direction.y = 0.0;
      direction.z = 0.0;
    }

    // Add new points extending the path
    double distance_added = 0.0;
    while (distance_added < path_extension_length_)
    {
      auto new_pose = last_pose;
      new_pose.position.x += direction.x * path_extension_resolution_;
      new_pose.position.y += direction.y * path_extension_resolution_;
      new_pose.position.z += direction.z * path_extension_resolution_;

      nav_msgs::msg::Path::_poses_type::value_type new_path_pose;
      new_path_pose.pose = new_pose;
      extended_path.poses.push_back(new_path_pose);

      distance_added += path_extension_resolution_;
      last_pose = new_pose;
    }

    return extended_path;
  }

  void checkObstacleProximity()
  {
    if (!current_pose_ || current_state_ != State::REFERENCE_PATH)
    {
      return;
    }

    // Get vehicle heading from orientation
    double vehicle_yaw = 0.0;
    tf2::Quaternion q(
        current_pose_->orientation.x,
        current_pose_->orientation.y,
        current_pose_->orientation.z,
        current_pose_->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, vehicle_yaw);

    // Vehicle forward vector
    double forward_x = std::cos(vehicle_yaw);
    double forward_y = std::sin(vehicle_yaw);

    bool obstacle_in_front = false;
    for (auto &obstacle : obstacle_positions_)
    {
      if (obstacle.passed)
        continue;

      double dx = obstacle.x - current_pose_->position.x;
      double dy = obstacle.y - current_pose_->position.y;
      double distance = std::hypot(dx, dy);

      // Check if obstacle is behind us
      double dot_product = dx * forward_x + dy * forward_y;
      if (dot_product < 0)
      {
        obstacle.passed = true;
        RCLCPP_WARN(this->get_logger(), "Marked obstacle at (%.2f, %.2f) as passed", obstacle.x, obstacle.y);
        continue;
      }

      // Only consider obstacles in front within trigger distance
      if (distance < trigger_distance_)
      {
        obstacle_in_front = true;
        break;
      }
    }

    // Remove passed obstacles
    obstacle_positions_.erase(
        std::remove_if(obstacle_positions_.begin(), obstacle_positions_.end(),
                       [](const Obstacle &o)
                       { return o.passed; }),
        obstacle_positions_.end());

    if (obstacle_in_front)
    {
      current_state_ = State::TRANSITION_TO_LEFT;
      transition_start_time_ = this->now();
      RCLCPP_WARN(this->get_logger(),
                  "Obstacle detected ahead! Switching to left lane\n"
                  "  Vehicle Position: (%.2f, %.2f)",
                  current_pose_->position.x,
                  current_pose_->position.y);
    }
  }

  void updatePath()
  {
    if (!right_threshold_ || !left_threshold_ || !ref_path_ ||
        right_threshold_->poses.empty() || left_threshold_->poses.empty() || ref_path_->poses.empty())
    {
      return;
    }

    reference_path_.poses.clear();
    reference_path_.header.frame_id = "map";
    reference_path_.header.stamp = this->now();

    size_t path_length = std::min({right_threshold_->poses.size(),
                                   left_threshold_->poses.size(),
                                   ref_path_->poses.size()});

    switch (current_state_)
    {
    case State::REFERENCE_PATH:
      for (size_t i = 0; i < path_length; i++)
      {
        reference_path_.poses.push_back(ref_path_->poses[i]);
      }
      break;

    case State::TRANSITION_TO_LEFT:
    {
      double elapsed = (this->now() - transition_start_time_).seconds();
      double ratio = std::min(elapsed / transition_duration_, 1.0);

      // Smooth easing function
      ratio = ratio < 0.5 ? 2 * ratio * ratio : 1 - std::pow(-2 * ratio + 2, 2) / 2;

      for (size_t i = 0; i < path_length; i++)
      {
        auto left_lane_pose = interpolatePose(
            left_threshold_->poses[i].pose,
            right_threshold_->poses[i].pose,
            left_lane_position_ratio_);
        auto pose = interpolatePose(ref_path_->poses[i].pose, left_lane_pose, ratio);
        reference_path_.poses.push_back(createPathPose(pose));
      }

      if (ratio >= 1.0)
      {
        current_state_ = State::LEFT_LANE;
        transition_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Now in left lane (avoiding obstacles)");
      }
      break;
    }

    case State::LEFT_LANE:
      for (size_t i = 0; i < path_length; i++)
      {
        auto pose = interpolatePose(
            left_threshold_->poses[i].pose,
            right_threshold_->poses[i].pose,
            left_lane_position_ratio_);
        reference_path_.poses.push_back(createPathPose(pose));
      }
      break;

    case State::TRANSITION_TO_REF:
    {
      double elapsed = (this->now() - transition_start_time_).seconds();
      double ratio = std::min(elapsed / return_transition_duration_, 1.0);

      // Faster linear transition for return
      for (size_t i = 0; i < path_length; i++)
      {
        auto left_lane_pose = interpolatePose(
            left_threshold_->poses[i].pose,
            right_threshold_->poses[i].pose,
            left_lane_position_ratio_);
        auto pose = interpolatePose(left_lane_pose, ref_path_->poses[i].pose, ratio);
        reference_path_.poses.push_back(createPathPose(pose));
      }

      if (ratio >= 1.0)
      {
        current_state_ = State::REFERENCE_PATH;
        RCLCPP_INFO(this->get_logger(), "Now back on reference path");
      }
      break;
    }
    }
  }

  geometry_msgs::msg::Pose interpolatePose(
      const geometry_msgs::msg::Pose &pose1,
      const geometry_msgs::msg::Pose &pose2,
      double ratio)
  {
    geometry_msgs::msg::Pose result;
    result.position.x = pose1.position.x + ratio * (pose2.position.x - pose1.position.x);
    result.position.y = pose1.position.y + ratio * (pose2.position.y - pose1.position.y);
    result.position.z = pose1.position.z + ratio * (pose2.position.z - pose1.position.z);
    result.orientation = pose1.orientation;
    return result;
  }

  nav_msgs::msg::Path::_poses_type::value_type createPathPose(
      const geometry_msgs::msg::Pose &pose)
  {
    nav_msgs::msg::Path::_poses_type::value_type path_pose;
    path_pose.pose = pose;
    return path_pose;
  }

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_threshold_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_threshold_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obstacles_sub_;

  // Publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr return_check_timer_;

  // Data storage
  nav_msgs::msg::Path::SharedPtr right_threshold_;
  nav_msgs::msg::Path::SharedPtr left_threshold_;
  nav_msgs::msg::Path::SharedPtr ref_path_;
  nav_msgs::msg::Path reference_path_;
  std::optional<geometry_msgs::msg::Pose> current_pose_;
  std::vector<Obstacle> obstacle_positions_;

  // State
  State current_state_;
  rclcpp::Time transition_start_time_;

  // Configuration
  double trigger_distance_;
  double avoidance_duration_;
  double transition_duration_;
  double return_transition_duration_;
  double left_lane_position_ratio_;
  double path_extension_length_;
  double path_extension_resolution_;
  double obstacle_min_separation_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneSwitchingPlanner>());
  rclcpp::shutdown();
  return 0;
}