#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class GPSWaypointFollower : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GPSWaypointFollower()
    : Node("gps_waypoint_follower"),
    lat_origin_(55.751244),
    lon_origin_(37.618423)
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        // Point B: ~8 m east of the origin (goal pad)
        const double deg_to_rad = M_PI / 180.0;
        const double meters_per_deg_lon = 111139.0 * std::cos(lat_origin_ * deg_to_rad);
        target_lat_ = lat_origin_;
        target_lon_ = lon_origin_ + (3.75 / meters_per_deg_lon);


        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
                                         std::bind(&GPSWaypointFollower::initiate_navigation, this));
    }

private:
    void initiate_navigation()
    {
        timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(30))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 'navigate_to_pose' not available!");
            return;
        }

        send_goal();
    }

    std::pair<double, double> gps_to_local_xy(double lat, double lon) const
    {
        constexpr double r_earth = 6378137.0;
        constexpr double deg_to_rad = M_PI / 180.0;

        double d_lat = (lat - lat_origin_) * deg_to_rad;
        double d_lon = (lon - lon_origin_) * deg_to_rad;
        double lat_ref = lat_origin_ * deg_to_rad;

        double x = d_lon * r_earth * std::cos(lat_ref);
        double y = d_lat * r_earth;
        return {x, y};
    }

    void send_goal()
    {
        auto [target_x, target_y] = gps_to_local_xy(target_lat_, target_lon_);
        RCLCPP_INFO(
            this->get_logger(),
                    "Target GPS converted to Cartesian frame 'map': X=%.2f m, Y=%.2f m",
                    target_x, target_y);

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = target_x;
        goal_msg.pose.pose.position.y = target_y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
        std::bind(&GPSWaypointFollower::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.result_callback =
        std::bind(&GPSWaypointFollower::result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending goal pose to Nav2...");
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the Nav2 action server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2. Moving to waypoint...");
        }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "SUCCESS: Robot reached the goal!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed or was canceled.");
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    double lat_origin_;
    double lon_origin_;
    double target_lat_;
    double target_lon_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSWaypointFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
