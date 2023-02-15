#include "run_mgr/run_mgr_node.hpp"

rm::RunMgrNode::RunMgrNode(const rclcpp::NodeOptions& options) : Node("run_mgr", options) {
    this->run_folder_sub = this->create_subscription<std_msgs::msg::String>(
        "/run_folder", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr str) { this->score_file_path = std::filesystem::path{str->data}; });

    this->score_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/score/rgb", rclcpp::QoS(10), std::bind(&rm::RunMgrNode::img_rcv_handler, this, std::placeholders::_1));

    this->order_66_pub = this->create_publisher<std_msgs::msg::UInt16>("/order_66", rclcpp::QoS(1).reliable());
}

void rm::RunMgrNode::img_rcv_handler(sensor_msgs::msg::Image::SharedPtr img) {
    //TODO
}
