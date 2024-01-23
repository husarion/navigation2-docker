#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;

#define LOOP_PERIOD 1s
#define SAVE_MAP_CONNECTION_TIMEOUT 2s
#define MIN_SAVE_MAP_PERIOD 5s

class HealthCheckNode : public rclcpp::Node {
public:
  HealthCheckNode()
      : Node("healthcheck_navigation"), map_exist(false),
        is_controller_active(false) {

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", qos,
        std::bind(&HealthCheckNode::mapCallback, this, std::placeholders::_1));

    controller_subscriber_ =
        create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            "controller_server/transition_event", rclcpp::SystemDefaultsQoS(),
            std::bind(&HealthCheckNode::controllerCallback, this,
                      std::placeholders::_1));

    save_map_client_ =
        create_client<nav2_msgs::srv::SaveMap>("map_saver/save_map");

    // Read the ROBOT_NAMESPACE environment variable
    const char *robotNamespaceEnv = std::getenv("ROBOT_NAMESPACE");
    ns_prefix = (robotNamespaceEnv != nullptr)
                    ? "/" + std::string(robotNamespaceEnv)
                    : "";
    if (!ns_prefix.empty()) {
      RCLCPP_INFO(get_logger(), "ROBOT_NAMESPACE: %s", ns_prefix.c_str());
    }

    // Read the SAVE_MAP_PERIOD environment variable
    const char *saveMapPeriodEnv = std::getenv("SAVE_MAP_PERIOD");
    if (saveMapPeriodEnv != nullptr) {
      try {
        saveMapPeriod = duration<double>(std::stod(saveMapPeriodEnv));
        if (saveMapPeriod < MIN_SAVE_MAP_PERIOD) {
          saveMapPeriod = MIN_SAVE_MAP_PERIOD;
        }
        RCLCPP_INFO(get_logger(), "SAVE_MAP_PERIOD: %.2lf seconds",
                    saveMapPeriod.count());
      } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR(get_logger(), "Invalid value for SAVE_MAP_PERIOD");
      }
    } else {
      saveMapPeriod = duration<double>(0s);
      RCLCPP_INFO(get_logger(), "SAVE_MAP_PERIOD environment variable not set. "
                                "Autosave save map disable.");
    }
  }

  void healthyCheck() {
    if (map_exist && is_controller_active) {
      writeHealthStatus("healthy");
    } else {
      writeHealthStatus("unhealthy");
    }
  }

  void saveMapPeriodically() {
    static steady_clock::time_point last_saved_map_time = steady_clock::now();
    auto elapsed_time = steady_clock::now() - last_saved_map_time;

    if (saveMapPeriod != 0s) {
      if (elapsed_time > saveMapPeriod) {
        if (save_map_client_->wait_for_service(SAVE_MAP_CONNECTION_TIMEOUT)) {
          RCLCPP_DEBUG(get_logger(), "map_saver/save_map service available");
          auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
          request->free_thresh = 0.25;
          request->occupied_thresh = 0.65;
          request->map_topic = ns_prefix + "/map";
          request->map_url = "/maps/map";
          request->map_mode = "trinary";
          request->image_format = "png";

          auto future = save_map_client_->async_send_request(request);

          if (rclcpp::spin_until_future_complete(shared_from_this(), future,
                                                 SAVE_MAP_CONNECTION_TIMEOUT) ==
              rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(get_logger(), "Map saved");
            last_saved_map_time = steady_clock::now();
          } else {
            RCLCPP_WARN(get_logger(),
                        "save_map service response didn't arrived");
          }
        } else {
          RCLCPP_DEBUG(get_logger(), "save_map service unavailable");
        }
      }
    }
  }

private:
  bool map_exist;
  bool is_controller_active;
  duration<double> saveMapPeriod;
  std::string ns_prefix;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
      controller_subscriber_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;

  void writeHealthStatus(const std::string &status) {
    std::ofstream healthFile("/var/tmp/health_status.txt");
    healthFile << status;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Map msg arrived");
    map_exist = true;
  }

  void controllerCallback(
      const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Controller msg arrived: %s",
                 msg->goal_state.label.c_str());
    if (msg->goal_state.label == "active") {
      is_controller_active = true;
    } else {
      is_controller_active = false;
    }
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
