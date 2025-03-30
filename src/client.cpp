#include "rclcpp/rclcpp.hpp"

#include <optional>
#include <chrono>
#include <Eigen/Geometry>

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
    : Node("get_rgbd_client")
    , logger_(this->get_logger())
    {
        const std::string srv_rgbd = "get_rgbd";
        client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        client_get_rgbd_ = this->create_client<segfault_pkg::srv::GetRGBD>(srv_rgbd, rmw_qos_profile_services_default, client_cb_group_);
        
        // wait for service to connect
        RCLCPP_INFO(logger_, "Waiting for service client to connect to server!");
        constexpr auto timeout = std::chrono::seconds(1);
        while (rclcpp::ok() && (!client_get_rgbd_->service_is_ready()))
        {
            client_get_rgbd_->wait_for_service(timeout);
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
    rclcpp::Client<segfault_pkg::srv::GetRGBD>::SharedPtr client_get_rgbd_;


    void callbackTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response>      res)
    {
        (void)req;

        RCLCPP_INFO(logger_, "Received TRIGGER request");
        res->success = false;

        // Get RGBD
        auto request = std::make_shared<segfault_pkg::srv::GetRGBD::Request>();
        RCLCPP_INFO(logger_, "CLIENT: Sending request ..");
        auto response = sync_send_request<segfault_pkg::srv::GetRGBD>(client_get_rgbd_, request).get();

        RCLCPP_INFO_STREAM(logger_, "CLIENT: RGBD and camera pose acquisition status: " << std::boolalpha << response->success);

        if (response->success)
        {
            RCLCPP_INFO_STREAM(logger_, "CLIENT: Size of color image height = " << response->rgb.height << ", width = " << response->rgb.width << ", encoding: " << response->rgb.encoding << ", row length = " << response->rgb.step);
            RCLCPP_INFO_STREAM(logger_, "CLIENT: Data size: " << response->rgb.data.size());
            RCLCPP_INFO_STREAM(logger_, "CLIENT: is_bigendian: " << response->rgb.is_bigendian);
            RCLCPP_INFO(logger_, "CLIENT: About to log header frame ID..");
            RCLCPP_INFO_STREAM(logger_, "CLIENT: header frame ID: " << response->rgb.header.frame_id);  // <- segfaults
            const sensor_msgs::msg::Image rgb_img = response->rgb;
            RCLCPP_INFO(logger_, "CLIENT: SUCCESS");
        }
        else
        {
            RCLCPP_WARN(logger_, "Try TRIGGER again!");
        }

        RCLCPP_INFO(logger_, "Completed TRIGGER request");
    }
};
