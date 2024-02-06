#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("publisher");
    auto publisher = node->create_publisher<std_msgs::msg::String>("chatter", 10);

    auto message = std_msgs::msg::String();
    message.data = "Hello, World!";

    rclcpp::WallRate loop_rate(2);  // Publish message every 2 seconds

    while (rclcpp::ok()) {
        publisher->publish(message);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
