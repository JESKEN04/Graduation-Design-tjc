#include <rclcpp/rclcpp.hpp>

class Node : public rclcpp::Node {
public:
    Node() : rclcpp::Node("node_name") {
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Node>());
    rclcpp::shutdown();
    return 0;
}
