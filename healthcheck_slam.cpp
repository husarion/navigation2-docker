#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "cstdlib"

using namespace std::chrono_literals;

#define TIMEOUT 3s

int msg_received = false;
int map_saved = false;

void msg_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::cout << "Message received" << std::endl;
  msg_received = true;
  if (msg_received && map_saved)
  {
    rclcpp::shutdown();
  }
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
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SensorDataQoS(),
                                                                msg_callback);
  auto client = node->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
  auto timer = node->create_wall_timer(TIMEOUT, timeout_callback);

  if (!client->wait_for_service(TIMEOUT))
  {
    std::cout << "Failed to connect to the image save service" << std::endl;
    rclcpp::shutdown();
  }

  auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
  request->free_thresh = 0.15;
  request->map_topic = "/map";
  request->map_url = "/maps/map";
  request->map_mode = "trinary";
  request->image_format = "png";

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout << "Response received" << std::endl;
    map_saved = true;
    if (msg_received && map_saved)
    {
      rclcpp::shutdown();
    }
  }

  rclcpp::spin(node);

  if (msg_received && map_saved)
  {
    return EXIT_SUCCESS;
  }
  else
  {
    return EXIT_FAILURE;
  }
}
