#ifndef MULTI_MAP_NAVIGATION_NAVIGATOR_H
#define MULTI_MAP_NAVIGATION_NAVIGATOR_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "multi_map_navigation/action/navigate_to.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include <sqlite3.h>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// A simple struct to hold wormhole data
struct WormholeData {
    geometry_msgs::msg::Pose entrance_pose;
    geometry_msgs::msg::Pose exit_pose;
};

class MultiMapNavigator : public rclcpp::Node
{
public:
    using NavigateToAction = multi_map_navigation::action::NavigateTo;
    using GoalHandleNavigateTo = rclcpp_action::ServerGoalHandle<NavigateToAction>;

    explicit MultiMapNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MultiMapNavigator();

private:
    // Action server and client
    rclcpp_action::Server<NavigateToAction>::SharedPtr action_server_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    // Service client for map loading
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;
    
    // Publisher for initial pose (AMCL)
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    // Database
    sqlite3* db_;
    std::string current_map_;

    // Action callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const NavigateToAction::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigateTo> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleNavigateTo> goal_handle);
    void execute(const std::shared_ptr<GoalHandleNavigateTo> goal_handle);

    // Helper functions
    bool getWormhole(const std::string& from_map, const std::string& to_map, WormholeData& wormhole);
    bool waitForService(rclcpp::ClientBase::SharedPtr client, std::chrono::seconds timeout);
};

#endif // MULTI_MAP_NAVIGATION_NAVIGATOR_H