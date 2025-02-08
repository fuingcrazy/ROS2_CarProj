
#include "Yubot_controller/ac_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>

namespace ac_controller
{
    void ac_controller::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        RCLCPP_INFO(logger_, "Configuring controller");
        auto node = parent.lock();      //returns a sharedptr if parent exists
        node->get_parameter_or("max_speed",max_speed_,6);
        node->get_parameter("lookahead_dist",lookahead_dist_,1.0);
        node->get_parameter("wheelbase",wheelbase_,0.54);
        tf_  = tf;
        costmap_ros_ = costmap_ros;
        name_ = name;
        logger_ = node->get_logger();
        global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);    //create path publisher

    }
    void ac_controller::cleanup()
    {
        RCLCPP_INFO(logger_, "Cleaning up controller");
        global_pub_.reset();
    }
    void ac_controller::activate()
    {
        RCLCPP_INFO(logger_, "Activating controller");
        global_pub_->on_activate();
    }
    void ac_controller::deactivate()
    {
        RCLCPP_INFO(logger_, "Deactivating controller");
        global_pub_->on_deactivate();
    }
    void PurePursuitController::setSpeedLimit(const double &speed_limit,
                                          const bool &percentage) {
  (void)speed_limit;
  (void)percentage;
}
void PurePursuitController::setPlan(const nav_msgs::msg::Path &path) {    //get global plan
  global_pub_->publish(path);
  global_plan_ = path;
}

 geometry_msgs::msg::TwistStamped computeVelocityCmd(const geometry_msgs::msg::PoseStamped& pose,
                                                      const geometry_msgs::msg::Twist& velocity,
                                                      nav2_core::GoalChecker* goal_checker)
    {
        (void)goal_checker;
        (void)velocity;
        geometry_msgs::msg::PoseStamped cmd_vel;
        cmd_vel.header.stamp = rclcpp.Clock().now();
        cmd_vel.frame_id = pose.header.frame_id;
        if(global_plan_.poses.empty()){
            RCLCPP_INFO(logger_,"No Path Received!");
            return cmd_vel;
        }
        auto transformed_plan = transformGlobalPlan(pose);    //transform global plan to local robot frame
        RCLCPP_INFO(logger_,"Local Plan generated, lenth: %d.",transformed_plan.poses.size());
        auto goal_it = std::find_if(transformed_plan.poses.begin(),transformed_plan.poses.end(),[&](const auto& ps){return hypot(ps.pose.position.x,ps.pose.position.y)>=lookahead_dist_; });
        
    }
}