#include "run_mgr/run_mgr_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<rm::RunMgrNode>(options);
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
