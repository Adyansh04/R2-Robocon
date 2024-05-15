#include "rclcpp/rclcpp.hpp"

class LineFollowingNode : public rclcpp::Node
{
    public:
        LineFollowingNode(): Node("line_following_node") {
            RCLCPP_INFO(this->get_logger(), "Line following node has been started");
        }

    private:
        // Add your private members and methods here
};