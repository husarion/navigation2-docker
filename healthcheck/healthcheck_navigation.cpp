#include "fstream"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

#define LOOP_PERIOD 2s

void write_health_status(const std::string &status) {
  std::ofstream healthFile("/health_status.txt");
  healthFile << status;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("healthcheck_node");
  std::string node_to_find = "/amcl";

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    auto node_names = node->get_node_names();
    auto is_node_running = std::find(node_names.begin(), node_names.end(),
                                     node_to_find) != node_names.end();

    if (is_node_running) {
      RCLCPP_INFO(node->get_logger(), "The %s node is running.",
                  node_to_find.c_str());
      write_health_status("healthy");

    } else {
      RCLCPP_WARN(node->get_logger(), "The %s node is not running.",
                  node_to_find.c_str());
      write_health_status("unhealthy");
    }
    std::this_thread::sleep_for(LOOP_PERIOD);
  }

  return 0;
}
