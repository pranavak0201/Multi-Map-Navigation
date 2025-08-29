#include "multi_map_navigation/navigator.h"
#include <thread>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::placeholders;

// C-style callback function for sqlite3_exec to process query results
static int db_callback(void* data, int argc, char** argv, char** azColName) {
    WormholeData* wormhole = static_cast<WormholeData*>(data);
    for (int i = 0; i < argc; i++) {
        if (std::string(azColName[i]) == "from_pos_x") wormhole->entrance_pose.position.x = atof(argv[i]);
        else if (std::string(azColName[i]) == "from_pos_y") wormhole->entrance_pose.position.y = atof(argv[i]);
        else if (std::string(azColName[i]) == "from_ori_z") wormhole->entrance_pose.orientation.z = atof(argv[i]);
        else if (std::string(azColName[i]) == "from_ori_w") wormhole->entrance_pose.orientation.w = atof(argv[i]);
        else if (std::string(azColName[i]) == "to_pos_x") wormhole->exit_pose.position.x = atof(argv[i]);
        else if (std::string(azColName[i]) == "to_pos_y") wormhole->exit_pose.position.y = atof(argv[i]);
        else if (std::string(azColName[i]) == "to_ori_z") wormhole->exit_pose.orientation.z = atof(argv[i]);
        else if (std::string(azColName[i]) == "to_ori_w") wormhole->exit_pose.orientation.w = atof(argv[i]);
    }
    return 0;
}

MultiMapNavigator::MultiMapNavigator(const rclcpp::NodeOptions & options) : Node("multi_map_navigator_node", options)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("db_path", "");
    this->declare_parameter<std::string>("initial_map", "map1");
    std::string db_path = this->get_parameter("db_path").as_string();
    current_map_ = this->get_parameter("initial_map").as_string();

    // Open the database
    if (sqlite3_open(db_path.c_str(), &db_)) {
        RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Opened database successfully at %s", db_path.c_str());
    }

    // Create clients and servers
    this->nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    this->action_server_ = rclcpp_action::create_server<NavigateToAction>(
        this, "navigate_to_map",
        std::bind(&MultiMapNavigator::handle_goal, this, _1, _2),
        std::bind(&MultiMapNavigator::handle_cancel, this, _1),
        std::bind(&MultiMapNavigator::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Multi Map Navigator Node has been started.");
}

MultiMapNavigator::~MultiMapNavigator() {
    sqlite3_close(db_);
    RCLCPP_INFO(this->get_logger(), "Database connection closed.");
}

// Helper function to query the database for a wormhole
bool MultiMapNavigator::getWormhole(const std::string& from_map, const std::string& to_map, WormholeData& wormhole) {
    std::string sql = "SELECT * FROM wormholes WHERE from_map_name = '" + from_map + "' AND to_map_name = '" + to_map + "';";
    char* zErrMsg = 0;
    int rc = sqlite3_exec(db_, sql.c_str(), db_callback, &wormhole, &zErrMsg);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "SQL error: %s", zErrMsg);
        sqlite3_free(zErrMsg);
        return false;
    }
    // Check if any data was actually loaded by the callback
    if (wormhole.entrance_pose.orientation.w == 0.0 && wormhole.entrance_pose.orientation.z == 0.0) {
        return false; // No entry found
    }
    return true;
}

// Standard action server callbacks
rclcpp_action::GoalResponse MultiMapNavigator::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToAction::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request for map '%s'", goal->target_map_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MultiMapNavigator::handle_cancel(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MultiMapNavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) {
    std::thread{std::bind(&MultiMapNavigator::execute, this, _1), goal_handle}.detach();
}

// The core execution logic
void MultiMapNavigator::execute(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<NavigateToAction::Result>();
    auto feedback = std::make_shared<NavigateToAction::Feedback>();
    
    const auto goal = goal_handle->get_goal();
    std::string target_map = goal->target_map_name;
    geometry_msgs::msg::PoseStamped final_target_pose = goal->target_pose;

    if (target_map == current_map_) {
        // --- SAME MAP NAVIGATION ---
        feedback->status = "Goal is on the current map. Navigating directly.";
        goal_handle->publish_feedback(feedback);
        
        auto nav2_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav2_goal.pose = final_target_pose;
        
        auto goal_handle_future = nav2_client_->async_send_goal(nav2_goal);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Send goal call failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        auto nav2_goal_handle = goal_handle_future.get();
        auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "Get result call failed");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        result->success = true;
        goal_handle->succeed(result);

    } else {
        // --- MULTI-MAP NAVIGATION ---
        feedback->status = "Goal is on a different map. Finding wormhole.";
        goal_handle->publish_feedback(feedback);

        WormholeData wormhole;
        if (!getWormhole(current_map_, target_map, wormhole)) {
            RCLCPP_ERROR(get_logger(), "No wormhole found from '%s' to '%s'", current_map_.c_str(), target_map.c_str());
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        feedback->status = "Navigating to wormhole entrance.";
        goal_handle->publish_feedback(feedback);

        // 1. Navigate to the wormhole entrance
        auto nav2_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav2_goal.pose.header.frame_id = "map";
        nav2_goal.pose.pose = wormhole.entrance_pose;
        
        auto goal_handle_future = nav2_client_->async_send_goal(nav2_goal);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future);
        auto nav2_goal_handle = goal_handle_future.get();
        if (!nav2_goal_handle) {
             RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
             result->success = false;
             goal_handle->abort(result);
             return;
        }
        auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

        // 2. Trigger the map switch
        feedback->status = "Reached wormhole. Switching maps...";
        goal_handle->publish_feedback(feedback);
        
        // Convert the exit pose's quaternion to yaw for the launch file
        tf2::Quaternion q(wormhole.exit_pose.orientation.x, wormhole.exit_pose.orientation.y, wormhole.exit_pose.orientation.z, wormhole.exit_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Construct the system command to launch the switching script
        std::stringstream ss;
        ss << "source ~/multi_map_ws/install/setup.bash && "
           << "ros2 launch multi_map_navigation switch_map.launch.py "
           << "map_name:=" << target_map << " "
           << "initial_pose_x:=" << std::fixed << std::setprecision(4) << wormhole.exit_pose.position.x << " "
           << "initial_pose_y:=" << std::fixed << std::setprecision(4) << wormhole.exit_pose.position.y << " "
           << "initial_pose_yaw:=" << std::fixed << std::setprecision(4) << yaw;

        RCLCPP_INFO(get_logger(), "Executing map switch command: %s", ss.str().c_str());
        std::system(ss.str().c_str());

        // Give time for new Nav2 to start up
        rclcpp::sleep_for(std::chrono::seconds(15));
        current_map_ = target_map;

        // 3. Navigate to the final goal in the new map
        feedback->status = "Map switched. Navigating to final goal.";
        goal_handle->publish_feedback(feedback);

        nav2_goal.pose = final_target_pose;
        goal_handle_future = nav2_client_->async_send_goal(nav2_goal);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future);
        nav2_goal_handle = goal_handle_future.get();
        if (!nav2_goal_handle) {
             RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server after map switch");
             result->success = false;
             goal_handle->abort(result);
             return;
        }
        result_future = nav2_client_->async_get_result(nav2_goal_handle);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);

        result->success = true;
        goal_handle->succeed(result);
    }
}