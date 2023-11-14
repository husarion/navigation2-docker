#include "rclcpp/rclcpp.hpp"
#include "bond/msg/status.hpp"
#include "cstdlib"

using namespace std::chrono_literals;

#define TIMEOUT 2s

int msg_received = EXIT_FAILURE;

void msg_callback(const bond::msg::Status::SharedPtr msg)
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
  auto sub = node->create_subscription<bond::msg::Status>("/bond", rclcpp::SystemDefaultsQoS(), msg_callback);
  auto timer = node->create_wall_timer(TIMEOUT, timeout_callback);

  rclcpp::spin(node);
  return msg_received;
}
