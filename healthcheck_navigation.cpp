#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create a ROS node
    auto node = std::make_shared<rclcpp::Node>("healthcheck_node");

    // Get a list of active node names
    auto node_names = node->get_node_names();

    bool bt_navigator_started = false;
    for (const auto &name : node_names) {
        if (name == "/bt_navigator") {
            bt_navigator_started = true;
            break;
        }
    }

    if (bt_navigator_started) {
        std::cout << "Node '/bt_navigator' has started." << std::endl;
    } else {
        std::cout << "Node '/bt_navigator' has not started yet." << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
