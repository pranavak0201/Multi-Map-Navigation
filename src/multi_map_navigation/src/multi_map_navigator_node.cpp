// In src/multi_map_navigator_node.cpp
#include "multi_map_navigation/navigator.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}