#include "rclcpp/rclcpp.hpp"

#include "server.cpp"
#include "client.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::Node::SharedPtr node_server = std::make_shared<GetRGBDServer>();
    rclcpp::Node::SharedPtr node_client = std::make_shared<GetRGBDClient>();
    
    rclcpp::executors::MultiThreadedExecutor mt_executor;
    mt_executor.add_node(node_server);
    mt_executor.add_node(node_client);
    mt_executor.spin();

    rclcpp::shutdown();

    return 0;
}