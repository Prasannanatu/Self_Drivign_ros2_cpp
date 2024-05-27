#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SelfDrivingNode : public rclcpp::Node
{
public:
    SelfDrivingNode() : Node("self_driving_node")
    {
        RCLCPP_INFO(this->get_logger(), "Self-driving node has been started.");
        // Add more initialization and functionality here
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SelfDrivingNode>());
    rclcpp::shutdown();
    return 0;
}
