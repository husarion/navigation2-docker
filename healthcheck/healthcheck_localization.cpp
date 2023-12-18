#include "fstream"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

void write_health_status(const std::string &status) {
  std::ofstream healthFile("/var/tmp/health_status.txt");
  healthFile << status;
}

void msg_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
  if (msg->goal_state.label == "active") {
    write_health_status("healthy");
  } else {
    write_health_status("unhealthy");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");

  auto sub = node->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      "amcl/transition_event", rclcpp::SystemDefaultsQoS(),
      msg_callback);

  rclcpp::spin(node);

  return 0;
}
