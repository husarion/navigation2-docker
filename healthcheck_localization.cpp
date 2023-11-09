#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cstdlib"

using namespace std::chrono_literals;

#define TIMEOUT 2s

int msg_received = EXIT_FAILURE;

void msg_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << "Message received" << std::endl;
  msg_received = EXIT_SUCCESS;
  rclcpp::shutdown();
}

void timeout_callback()
{
  std::cout << "Timeout" << std::endl;
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  rclcpp::QoS qos(1);
  qos.transient_local();
  auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", qos, msg_callback);
  auto timer = node->create_wall_timer(TIMEOUT, timeout_callback);

  rclcpp::spin(node);
  return msg_received;
}

