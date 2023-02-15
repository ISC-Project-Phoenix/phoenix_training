#pragma once
#include <filesystem>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "run_mgr/score_mgr.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace rm {
class RunMgrNode : public rclcpp::Node {
    /// Run folder subscription
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_folder_sub;

    /// Score camera subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr score_img_sub;

    /// Publisher to indicate we are done with this run
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr order_66_pub;

    /// Score algorithm
    std::shared_ptr<IScoreMgr> score_mgr;

    /// Path to score file we will write. None until we receive a message from /run_folder
    std::optional<std::filesystem::path> score_file_path;

public:
    RunMgrNode(const rclcpp::NodeOptions& options);

    void img_rcv_handler(sensor_msgs::msg::Image::SharedPtr img);
};
}  // namespace rm