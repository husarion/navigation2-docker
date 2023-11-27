#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono_literals;

#define LOOP_PERIOD 2s
#define MSG_VALID_TIME 5s
#define SAVE_MAP_PERIOD 15s
#define SAVE_MAP_CONNECTION_TIMEOUT 2s
#define SAVE_MAP_VALID_TIME 20s

class HealthCheckNode : public rclcpp::Node {
public:
  HealthCheckNode() : Node("healthcheck_node") {
    rclcpp::QoS qos(1);
    qos.transient_local();
    map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&HealthCheckNode::msgCallback, this, std::placeholders::_1));
    save_map_client_ =
        create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
  }

  void healthyCheck() {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> msg_elapsed_time =
        current_time - last_msg_time;
    std::chrono::duration<double> saved_map_elapsed_time =
        current_time - last_saved_map_time;
    bool is_msg_valid = msg_elapsed_time.count() < MSG_VALID_TIME.count();
    bool is_save_map_valid =
        saved_map_elapsed_time.count() < SAVE_MAP_VALID_TIME.count();

    if (is_msg_valid && is_save_map_valid) {
      writeHealthStatus("healthy");
    } else {
      writeHealthStatus("unhealthy");
    }
  }

  void saveMapPeriodically() {
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time =
        current_time - last_saved_map_time;

    if (elapsed_time.count() > SAVE_MAP_PERIOD.count()) {
      if (save_map_client_->wait_for_service(SAVE_MAP_CONNECTION_TIMEOUT)) {
        RCLCPP_DEBUG(get_logger(), "Service available");
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->free_thresh = 0.15;
        request->map_topic = "/map";
        request->map_url = "/maps/map";
        request->map_mode = "trinary";
        request->image_format = "png";

        auto future = save_map_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(shared_from_this(), future,
                                               SAVE_MAP_CONNECTION_TIMEOUT) ==
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_DEBUG(get_logger(), "Map saved");
          last_saved_map_time = std::chrono::steady_clock::now();
        } else {
          RCLCPP_DEBUG(get_logger(), "Service response didn't arrived");
        }
      } else {
        RCLCPP_DEBUG(get_logger(), "Service unavailable");
      }
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  std::chrono::steady_clock::time_point last_msg_time;
  std::chrono::steady_clock::time_point last_saved_map_time;

  void writeHealthStatus(const std::string &status) {
    std::ofstream healthFile("/health_status.txt");
    healthFile << status;
  }

  void msgCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Msg arrived");
    last_msg_time = std::chrono::steady_clock::now();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HealthCheckNode>();

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->healthyCheck();
    node->saveMapPeriodically();
    std::this_thread::sleep_for(LOOP_PERIOD);
  }

  return 0;
}
