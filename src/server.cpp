#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include <iostream>
#include <optional>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "segfault_pkg/srv/get_rgbd.hpp"


class GetRGBDServer : public rclcpp::Node
{
public:
    GetRGBDServer()
        : Node("get_rgbd_server")
        , logger_(this->get_logger())
        , rgb_topic_("/camera/camera/color/image_raw")
        , depth_topic_("/camera/camera/depth/image_rect_raw")
        , cam_info_topic_("/camera/camera/color/camera_info")
        , cam_info_(std::nullopt)
        , last_rgb_(std::nullopt)
        , last_depth_(std::nullopt)
        , first_rgbd_obtained_(false)
    {
        service_get_rgbd_ = this->create_service<segfault_pkg::srv::GetRGBD>("get_rgbd", std::bind(&GetRGBDServer::callbackGetRGBD, this, std::placeholders::_1, std::placeholders::_2));

        const uint32_t queue_size = 10;

        // cross-reference against realsense QoS with `ros2 topic info --verbose <topic_name>`
        rclcpp::SensorDataQoS qos;
        qos.keep_last(queue_size);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        subscriber_rgb_.subscribe(this, rgb_topic_, qos.get_rmw_qos_profile());
        subscriber_depth_.subscribe(this, depth_topic_, qos.get_rmw_qos_profile());

        approx_time_sync_ptr_ = std::make_shared<message_filters::Synchronizer<ApproxTimePolicy>>(ApproxTimePolicy(queue_size), subscriber_rgb_, subscriber_depth_);
        approx_time_sync_ptr_->registerCallback(std::bind(&GetRGBDServer::callbackApproxTimeSync, this, std::placeholders::_1, std::placeholders::_2));

        cam_info_ = std::nullopt;
        subscriber_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(cam_info_topic_, queue_size, std::bind(&GetRGBDServer::callbackCamInfo, this, std::placeholders::_1));

        debug_subscriber_rgb_ = this->create_subscription<sensor_msgs::msg::Image>(rgb_topic_, queue_size, std::bind(&GetRGBDServer::callbackRGB, this, std::placeholders::_1));
        debug_subscriber_depth_ = this->create_subscription<sensor_msgs::msg::Image>(depth_topic_, queue_size, std::bind(&GetRGBDServer::callbackDepth, this, std::placeholders::_1));

        RCLCPP_INFO(logger_, "Service get_rgbd_server online!");
    }

private:
    rclcpp::Logger logger_;

    const std::string rgb_topic_;
    const std::string depth_topic_;
    
    const std::string cam_info_topic_;
    std::optional<sensor_msgs::msg::CameraInfo> cam_info_;

    std::optional<sensor_msgs::msg::Image> last_rgb_;
    std::optional<sensor_msgs::msg::Image> last_depth_;
    std::optional<std_msgs::msg::Header> last_rgbd_timestamp_;
    bool first_rgbd_obtained_;

    rclcpp::Service<segfault_pkg::srv::GetRGBD>::SharedPtr service_get_rgbd_;
    message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_rgb_;
    message_filters::Subscriber<sensor_msgs::msg::Image> subscriber_depth_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproxTimePolicy;
    std::shared_ptr<message_filters::Synchronizer<ApproxTimePolicy>> approx_time_sync_ptr_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_cam_info_;

    // for debugging (helps to know if RGBD messages are coming in or not)
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_subscriber_rgb_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr debug_subscriber_depth_;


    void callbackApproxTimeSync(const sensor_msgs::msg::Image::ConstSharedPtr& img_rgb, const sensor_msgs::msg::Image::ConstSharedPtr& img_depth)
    {
        first_rgbd_obtained_ = true;
        RCLCPP_INFO_ONCE(logger_, "Acquired first RGBD frame");

        last_rgb_ = *img_rgb;
        last_depth_ = *img_depth;

        // ROS2 timestamp comparison: https://github.com/ros2/ros2/issues/1371
        if (rclcpp::Time(img_rgb->header.stamp) > rclcpp::Time(img_depth->header.stamp))
        {
            last_rgbd_timestamp_ = img_rgb->header;
        }
        else
        {
            last_rgbd_timestamp_ = img_depth->header;
        }
        RCLCPP_INFO_STREAM_ONCE(logger_, "First RGBD header frame_id: " << last_rgbd_timestamp_->frame_id);
    }

    void callbackRGB([[maybe_unused]] const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(logger_, "First color image rcvd!");
    }

    void callbackDepth([[maybe_unused]] const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(logger_, "First depth image rcvd!");
    }

    void callbackCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        RCLCPP_INFO_ONCE(logger_, "Acquired camera parameters!");
        cam_info_ = *msg;
    }

    void callbackGetRGBD(const std::shared_ptr<segfault_pkg::srv::GetRGBD::Request> req, std::shared_ptr<segfault_pkg::srv::GetRGBD::Response>      res)
    {
        (void)req;

        RCLCPP_INFO(logger_, "Service get_rgbd was called!");
        res->success = false;

        // return early if no RGBD on record
        if (!first_rgbd_obtained_)
        {
            RCLCPP_ERROR(logger_, "No RGBD received so far");
            return;
        }

        if (last_rgbd_timestamp_.has_value())
        {

            res->success = true;
            res->rgb = last_rgb_.value();
            res->depth = last_depth_.value();
            RCLCPP_INFO_STREAM(logger_, "SERVER: Assigned RGB and Depth, height = " << res->rgb.height << ", width = " << res->rgb.width << ", encoding: " << res->rgb.encoding << ", row length = " << res->rgb.step);
            RCLCPP_INFO_STREAM(logger_, "SERVER: Data size: " << res->rgb.data.size());
            RCLCPP_INFO_STREAM(logger_, "SERVER: Frame ID: " << res->rgb.header.frame_id);
            RCLCPP_INFO_STREAM(logger_, "SERVER: size of color image height = " << res->rgb.height << ", width = " << res->rgb.width << ", encoding: " << res->rgb.encoding << ", row length = " << res->rgb.step);
            RCLCPP_INFO_STREAM(logger_, "SERVER: is_bigendian: " << res->rgb.is_bigendian);

            // camera info
            if (cam_info_.has_value())
            {
                res->intrinsics = cam_info_.value();
            }
            else
            {
                RCLCPP_ERROR(logger_, "Camera intrinsics not yet received!");
                res->success = false;
                return;
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(logger_, "Did not receive color and depth image on topic " << rgb_topic_ << " and " << depth_topic_);
            return;
        }

        RCLCPP_INFO_STREAM(logger_, "Completed get_rgbd service request, frame ID: " << res->rgb.header.frame_id);
    }
};