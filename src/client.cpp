#include "rclcpp/rclcpp.hpp"

#include <chrono>

#include "segfault_pkg/srv/get_rgbd.hpp"

#include "std_srvs/srv/trigger.hpp"

template <typename T>
std::shared_ptr<typename T::Response> sync_send_request(
    typename rclcpp::Client<T>::SharedPtr client, 
    typename std::shared_ptr<typename T::Request> request
)
{
    return client->async_send_request(request).get();
}

class GetRGBDClient : public rclcpp::Node
{
public:
    GetRGBDClient()
    : Node("string_client")
    , logger_(this->get_logger())
    {
        const std::string srv_rgbd = "string";
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        client_string_ = this->create_client<segfault_pkg::srv::GetRGBD>(srv_rgbd, rmw_qos_profile_services_default, client_cb_group_);
        
        // wait for service to connect
        RCLCPP_INFO(logger_, "Waiting for service client to connect to server!");
        constexpr auto timeout = std::chrono::seconds(1);
        while (rclcpp::ok() && (!client_string_->service_is_ready()))
        {
            client_string_->wait_for_service(timeout);
        }
        RCLCPP_INFO(logger_, "Service client has connected to server!");
        
        // a Trigger service server to make it easy to make a CLI request which then makes the eponymous client make a request to the server defined in server.cpp
        srv_cli_trigger = this->create_service<std_srvs::srv::Trigger>("trigger", std::bind(&GetRGBDClient::callbackTrigger, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(logger_, "\033[1;32mReady to accept trigger request!\033[0m");
    }

private:
    rclcpp::Logger logger_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cli_trigger;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Client<segfault_pkg::srv::GetRGBD>::SharedPtr client_string_;


    void callbackTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response>      res)
    {
        (void)req;

        RCLCPP_INFO(logger_, "Received TRIGGER request");
        res->success = false;

        // Get RGBD
        auto request = std::make_shared<segfault_pkg::srv::GetRGBD::Request>();
        request->input.data = "Hello there!";
        RCLCPP_INFO(logger_, "CLIENT: Sending request ..");
        auto response = sync_send_request<segfault_pkg::srv::GetRGBD>(client_string_, request).get();

        RCLCPP_INFO_STREAM(logger_, "CLIENT: RGBD and camera pose acquisition status: " << std::boolalpha << response->success);

        if (response->success)
        {
            RCLCPP_INFO(logger_, "CLIENT: About to log output data ..");
            RCLCPP_INFO_STREAM(logger_, "CLIENT: txt: " << response->output.data);
            RCLCPP_INFO(logger_, "CLIENT: SUCCESS");
        }
        else
        {
            RCLCPP_WARN(logger_, "Try TRIGGER again!");
        }

        RCLCPP_INFO(logger_, "Completed TRIGGER request");
    }
};
