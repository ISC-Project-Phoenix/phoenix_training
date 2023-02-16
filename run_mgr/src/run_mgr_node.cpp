#include "run_mgr/run_mgr_node.hpp"

#include <fstream>

#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

rm::RunMgrNode::RunMgrNode(const rclcpp::NodeOptions& options) : Node("run_mgr", options) {
    this->run_folder_sub = this->create_subscription<std_msgs::msg::String>(
        "/run_folder", rclcpp::QoS(1).reliable().transient_local(), [this](std_msgs::msg::String::SharedPtr str) {
            this->score_file_path = std::filesystem::path{str->data} / "score.txt";
            RCLCPP_INFO(this->get_logger(), "Set score file path to: %s", this->score_file_path->c_str());
        });

    this->score_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/score/rgb", rclcpp::SensorDataQoS{},
        std::bind(&rm::RunMgrNode::img_rcv_handler, this, std::placeholders::_1));

    this->order_66_pub = this->create_publisher<std_msgs::msg::UInt16>("/order_66", rclcpp::QoS(1).reliable());

    this->score_mgr = std::make_shared<rm::BasicScoreMgr>();
}

void rm::RunMgrNode::img_rcv_handler(sensor_msgs::msg::Image::SharedPtr img) {
    // On first image received, create a timer that lowers score over time. This serves as a way to not decrease the score until sim is up
    static bool first_img = true;
    if (first_img) {
        RCLCPP_INFO(this->get_logger(), "Detected sim up, starting score timer...");
        this->timer = this->create_wall_timer(1000ms, [this]() { this->score_mgr->dec_score(1); });
        first_img = false;
    }

    // Force bgr8 since gazebo gives rgb8 for some reason
    const auto cv_img = cv_bridge::toCvShare(img, "bgr8");
    const auto mat = cv_img->image;

    // Take action on score depending on our state
    auto state = this->score_mgr->determine_state(mat);
    bool should_finish = false;
    switch (state) {
        case IScoreMgr::State::InBounds:
            break;
        case IScoreMgr::State::OutOfBounds:
            RCLCPP_INFO(this->get_logger(), "Kart out of bounds!");
            this->score_mgr->out_of_bounds(this->get_clock()->now());
            break;
        case IScoreMgr::State::FinishLine:
            RCLCPP_INFO(this->get_logger(), "Detected finish line!");
            should_finish = true;
            break;
    }

    if (should_finish || this->score_mgr->get_score() == 0) {
        auto final_score = this->score_mgr->get_score();

        this->finish_run(final_score);
    }
}

void rm::RunMgrNode::finish_run(uint16_t final_score) {
    // Write final score to file
    if (!this->score_file_path.has_value()) {
        RCLCPP_FATAL(this->get_logger(),
                     "run_mgr finished run but never got a run folder path! Unable to write score.");
    } else {
        auto score_file = std::ofstream{*this->score_file_path};
        score_file << std::to_string(final_score);
    }

    RCLCPP_INFO(this->get_logger(), "Run finished with score %d", final_score);
    auto score_msg = std_msgs::msg::UInt16{};
    score_msg.data = final_score;

    // Commander Cody, the time has come.
    this->order_66_pub->publish(score_msg);

    RCLCPP_INFO(this->get_logger(), "Sent order66");

    // Block until command to kill is received to avoid needing to debounce this function.
    this->order_66_pub->wait_for_all_acked();

    // Make sure this node doesn't continue after this function, if for some reason the sigterm doesn't kill
    std::terminate();
}
