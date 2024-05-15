#include "rclcpp/rclcpp.hpp"
#include "line_following/line_follower_header.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineFollowingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
