#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include "segfault_pkg/srv/get_string.hpp"


class GetStringServer : public rclcpp::Node
{
public:
    GetStringServer()
        : Node("string_server")
        , logger_(this->get_logger())
    {
        service_string_ = this->create_service<segfault_pkg::srv::GetString>("string", std::bind(&GetStringServer::callbackGetString, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(logger_, "Service string_server online!");
    }

private:
    rclcpp::Logger logger_;

    rclcpp::Service<segfault_pkg::srv::GetString>::SharedPtr service_string_;

    void callbackGetString(const std::shared_ptr<segfault_pkg::srv::GetString::Request> req, std::shared_ptr<segfault_pkg::srv::GetString::Response>      res)
    {
        RCLCPP_INFO_STREAM(logger_, "SERVER: Service string was called! Txt: " << req->input.data);
        res->success = true;
        res->output.data = "General Kenobi!!";
        RCLCPP_INFO_STREAM(logger_, "SERVER: Completed string service request, responding with " << res->output.data);
    }
};