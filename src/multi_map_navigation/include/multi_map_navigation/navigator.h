#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp" // For the Nav2 client
#include "multi_map_navigation/action/navigate_to.hpp"
#include <sqlite3.h> // For the database connection
#include "geometry_msgs/msg/pose.hpp"

// A simple struct to hold wormhole data retrieved from the database
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
    ~MultiMapNavigator(); // Add a destructor to close the database

private:
    // Your action server
    rclcpp_action::Server<NavigateToAction>::SharedPtr action_server_;

    // Action client to talk to Nav2
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;

    // Database connection handle
    sqlite3* db_;
    std::string current_map_;

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateTo> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateTo> goal_handle);

    void execute(const std::shared_ptr<GoalHandleNavigateTo> goal_handle);
    
    // Helper function to query the database
    bool getWormhole(const std::string& from_map, const std::string& to_map, WormholeData& wormhole);
};

#endif // NAVIGATOR_H