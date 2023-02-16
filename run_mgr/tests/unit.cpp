#include "gtest/gtest.h"
#include "run_mgr/run_mgr_node.hpp"

TEST(ScoreMgrTests, StateDetection) {
    auto in_bounds = cv::imread("test_assets/in_bounds.png");
    auto mostly_oob = cv::imread("test_assets/mostly_oob.png");
    auto on_finish = cv::imread("test_assets/on_finish.png");

    auto score_mgr = rm::BasicScoreMgr{};

    EXPECT_EQ(score_mgr.determine_state(in_bounds), rm::IScoreMgr::State::InBounds);
    EXPECT_EQ(score_mgr.determine_state(mostly_oob), rm::IScoreMgr::State::OutOfBounds);
    EXPECT_EQ(score_mgr.determine_state(on_finish), rm::IScoreMgr::State::FinishLine);
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}