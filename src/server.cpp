#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <optional>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "segfault_pkg/srv/get_rgbd.hpp"


class GetRGBDServer : public rclcpp::Node
{
public:
    GetRGBDServer()
        : Node("string_server")
        , logger_(this->get_logger())
    {
        service_string_ = this->create_service<segfault_pkg::srv::GetRGBD>("string", std::bind(&GetRGBDServer::callbackGetRGBD, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(logger_, "Service string_server online!");
    }

private:
    rclcpp::Logger logger_;

    rclcpp::Service<segfault_pkg::srv::GetRGBD>::SharedPtr service_string_;

    void callbackGetRGBD(const std::shared_ptr<segfault_pkg::srv::GetRGBD::Request> req, std::shared_ptr<segfault_pkg::srv::GetRGBD::Response>      res)
    {
        RCLCPP_INFO_STREAM(logger_, "SERVER: Service string was called! Txt: " << req->input.data);
        res->success = true;
        res->output.data = "General Kenobi!!";
        RCLCPP_INFO_STREAM(logger_, "SERVER: Completed string service request, responding with " << res->output.data);
    }
};