#include "multi_map_navigation/navigator.h"
#include <thread>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::placeholders;

// (db_callback function remains the same)
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
    // ... (Constructor is the same as the last version) ...
    this->declare_parameter<std::string>("db_path", "");
    this->declare_parameter<std::string>("initial_map", "map3");
    std::string db_path = this->get_parameter("db_path").as_string();
    current_map_ = this->get_parameter("initial_map").as_string();
    if (sqlite3_open(db_path.c_str(), &db_)) {
        RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db_));
    } else {
        RCLCPP_INFO(this->get_logger(), "Opened database successfully at %s", db_path.c_str());
    }
    this->nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    this->action_server_ = rclcpp_action::create_server<NavigateToAction>(
        this, "navigate_to_map",
        std::bind(&MultiMapNavigator::handle_goal, this, _1, _2),
        std::bind(&MultiMapNavigator::handle_cancel, this, _1),
        std::bind(&MultiMapNavigator::handle_accepted, this, _1));

    // Launch the initial Nav2 stack
    RCLCPP_INFO(get_logger(), "Performing initial launch of Nav2 for %s...", current_map_.c_str());
    std::string pkg_share_path = ament_index_cpp::get_package_share_directory("multi_map_navigation");
    std::stringstream initial_launch_cmd;
    initial_launch_cmd << "bash -c \". ~/multi_map_ws/install/setup.bash && "
                       << "ros2 launch nav2_bringup bringup_launch.py "
                       << "map:=" << pkg_share_path << "/maps/" << current_map_ << ".yaml "
                       << "use_sim_time:=True "
                       << "params_file:=" << pkg_share_path << "/config/nav2_params.yaml "
                       << "autostart:=True & "
                       << "sleep 5 && ros2 launch nav2_bringup rviz_launch.py &\"";
    std::system(initial_launch_cmd.str().c_str());

    RCLCPP_INFO(this->get_logger(), "Multi Map Navigator Node has been started and initial Nav2 launch command sent.");
}

// (Destructor, getWormhole, and other callbacks remain the same)
MultiMapNavigator::~MultiMapNavigator() { sqlite3_close(db_); }
bool MultiMapNavigator::getWormhole(const std::string& from_map, const std::string& to_map, WormholeData& wormhole) {
    std::string sql = "SELECT * FROM wormholes WHERE from_map_name = '" + from_map + "' AND to_map_name = '" + to_map + "';";
    char* zErrMsg = 0;
    int rc = sqlite3_exec(db_, sql.c_str(), db_callback, &wormhole, &zErrMsg);
    if (rc != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "SQL error: %s", zErrMsg);
        sqlite3_free(zErrMsg);
        return false;
    }
    if (wormhole.entrance_pose.orientation.w == 0.0 && wormhole.entrance_pose.orientation.z == 0.0) { return false; }
    return true;
}
rclcpp_action::GoalResponse MultiMapNavigator::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToAction::Goal> goal) { (void)uuid; (void)goal; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
rclcpp_action::CancelResponse MultiMapNavigator::handle_cancel(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) { (void)goal_handle; return rclcpp_action::CancelResponse::ACCEPT; }
void MultiMapNavigator::handle_accepted(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) { std::thread{std::bind(&MultiMapNavigator::execute, this, _1), goal_handle}.detach(); }


void MultiMapNavigator::execute(const std::shared_ptr<GoalHandleNavigateTo> goal_handle) {
    auto result = std::make_shared<NavigateToAction::Result>();
    const auto goal = goal_handle->get_goal();
    std::string target_map = goal->target_map_name;
    geometry_msgs::msg::PoseStamped final_target_pose = goal->target_pose;

    if (target_map == current_map_) {
        // --- SAME MAP NAVIGATION --- (Unchanged)
        auto nav2_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav2_goal.pose = final_target_pose;
        auto nav2_goal_handle = nav2_client_->async_send_goal(nav2_goal).get();
        if (!nav2_goal_handle) { result->success = false; goal_handle->abort(result); return; }
        auto nav2_result = nav2_client_->async_get_result(nav2_goal_handle).get();
        result->success = (nav2_result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (result->success) goal_handle->succeed(result); else goal_handle->abort(result);
    } else {
        // --- MULTI-MAP NAVIGATION ---
        WormholeData wormhole;
        if (!getWormhole(current_map_, target_map, wormhole)) {
            RCLCPP_ERROR(get_logger(), "No wormhole found from '%s' to '%s'", current_map_.c_str(), target_map.c_str());
            result->success = false; goal_handle->abort(result); return;
        }

        // 1. Navigate to the wormhole
        auto nav2_goal = nav2_msgs::action::NavigateToPose::Goal();
        nav2_goal.pose.header.frame_id = "map";
        nav2_goal.pose.pose = wormhole.entrance_pose;
        auto nav2_goal_handle = nav2_client_->async_send_goal(nav2_goal).get();
        if (!nav2_goal_handle || nav2_client_->async_get_result(nav2_goal_handle).get().code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(), "Failed to navigate to wormhole");
            result->success = false; goal_handle->abort(result); return;
        }

        // --- NEW, ROBUST KILL LOGIC ---
        RCLCPP_INFO(get_logger(), "Reached wormhole. Killing old Nav2 and RViz nodes...");
        std::system("pkill -f nav2_bringup");
        std::system("pkill -f rviz2");
        rclcpp::sleep_for(std::chrono::seconds(5));
        // --- END OF NEW LOGIC ---
        
        // 3. Directly launch the new Nav2 and RViz stacks
        tf2::Quaternion q(wormhole.exit_pose.orientation.x, wormhole.exit_pose.orientation.y, wormhole.exit_pose.orientation.z, wormhole.exit_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        std::string pkg_share_path = ament_index_cpp::get_package_share_directory("multi_map_navigation");
        std::stringstream launch_cmd;
        launch_cmd << "bash -c \". ~/multi_map_ws/install/setup.bash && "
                   << "ros2 launch nav2_bringup bringup_launch.py "
                   << "map:=" << pkg_share_path << "/maps/" << target_map << ".yaml "
                   << "use_sim_time:=True "
                   << "params_file:=" << pkg_share_path << "/config/nav2_params.yaml "
                   << "autostart:=True "
                   << "initial_pose.x:=" << std::fixed << std::setprecision(4) << wormhole.exit_pose.position.x << " "
                   << "initial_pose.y:=" << std::fixed << std::setprecision(4) << wormhole.exit_pose.position.y << " "
                   << "initial_pose.yaw:=" << std::fixed << std::setprecision(4) << yaw << " & "
                   << "sleep 5 && ros2 launch nav2_bringup rviz_launch.py &\"";

        RCLCPP_INFO(get_logger(), "Executing map switch command: %s", launch_cmd.str().c_str());
        std::system(launch_cmd.str().c_str());

        current_map_ = target_map;

        RCLCPP_INFO(get_logger(), "Waiting for new Nav2 action server to become available...");
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(30))) {
            RCLCPP_ERROR(get_logger(), "Nav2 action server did not become available after map switch.");
            result->success = false; goal_handle->abort(result); return;
        }

        // 4. Navigate to the final goal
        nav2_goal.pose = final_target_pose;
        nav2_goal_handle = nav2_client_->async_send_goal(nav2_goal).get();
        if (!nav2_goal_handle) {
             RCLCPP_ERROR(get_logger(), "Final goal was rejected by server after map switch");
             result->success = false; goal_handle->abort(result); return;
        }
        auto nav2_result_final = nav2_client_->async_get_result(nav2_goal_handle).get();
        
        result->success = (nav2_result_final.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (result->success) goal_handle->succeed(result); else goal_handle->abort(result);
    }
}