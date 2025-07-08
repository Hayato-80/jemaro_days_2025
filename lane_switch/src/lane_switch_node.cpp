#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class PathAlternator : public rclcpp::Node
{
public:
  PathAlternator() : Node("path_alternator"), current_source_(Source::RIGHT)
  {
    // Create subscribers
    right_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path_right", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
          right_path_ = msg;
        });

    left_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path_left", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
          left_path_ = msg;
        });

    // Create publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create timer for switching between sources
    timer_ = this->create_wall_timer(
        5s, [this]()
        { this->switchSource(); });

    // Create timer for publishing at a fixed rate
    publish_timer_ = this->create_wall_timer(
        100ms, [this]()
        { this->publishCurrentPath(); });

    RCLCPP_INFO(this->get_logger(), "Path alternator node initialized");
  }

private:
  enum class Source
  {
    RIGHT,
    LEFT
  };

  void switchSource()
  {
    if (current_source_ == Source::RIGHT)
    {
      current_source_ = Source::LEFT;
      RCLCPP_INFO(this->get_logger(), "Switched to LEFT path source");
    }
    else
    {
      current_source_ = Source::RIGHT;
      RCLCPP_INFO(this->get_logger(), "Switched to RIGHT path source");
    }
  }

  void publishCurrentPath()
  {
    if (current_source_ == Source::RIGHT && right_path_)
    {
      path_pub_->publish(*right_path_);
    }
    else if (current_source_ == Source::LEFT && left_path_)
    {
      path_pub_->publish(*left_path_);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  nav_msgs::msg::Path::SharedPtr right_path_;
  nav_msgs::msg::Path::SharedPtr left_path_;
  Source current_source_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathAlternator>());
  rclcpp::shutdown();
  return 0;
}