#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "fstream"

using namespace std::chrono_literals;

#define LOOP_PERIOD 2s

// State 'healthy' is latched, because topic 'amcl_pose' is transient_local and send only when moving.

void write_health_status(const std::string& status)
{
  std::ofstream healthFile("/health_status.txt");
  healthFile << status;
}

void msg_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << "Message received" << std::endl;
  write_health_status("healthy");
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  rclcpp::QoS qos(1);
  qos.transient_local();
  auto sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", qos, msg_callback);

  write_health_status("unhealthy");
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(LOOP_PERIOD);
  }

  return 0;
}
