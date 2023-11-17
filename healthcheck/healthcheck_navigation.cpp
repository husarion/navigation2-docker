#include "bond/msg/status.hpp"
#include "fstream"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define LOOP_PERIOD 2s
#define MSG_VALID_TIME 5s

std::chrono::steady_clock::time_point last_msg_time;

void write_health_status(const std::string &status) {
  std::ofstream healthFile("/health_status.txt");
  healthFile << status;
}

void msg_callback(const bond::msg::Status::SharedPtr msg) {
  std::cout << "Message received" << std::endl;
  last_msg_time = std::chrono::steady_clock::now();
}

void healthy_check(const rclcpp::Node::SharedPtr &node) {
  std::chrono::steady_clock::time_point current_time =
      std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - last_msg_time;
  bool is_msg_valid = elapsed_time.count() < MSG_VALID_TIME.count();

  if (is_msg_valid) {
    RCLCPP_INFO(node->get_logger(), "Health check: healthy");
    write_health_status("healthy");
  } else {
    RCLCPP_WARN(node->get_logger(), "Health check: unhealthy");
    write_health_status("unhealthy");
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  auto sub = node->create_subscription<bond::msg::Status>(
      "/bond", rclcpp::SystemDefaultsQoS(), msg_callback);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    healthy_check(node);
    std::this_thread::sleep_for(LOOP_PERIOD);
  }

  return 0;
}
