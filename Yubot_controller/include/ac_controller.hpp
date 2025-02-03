#ifndef AC_CONTROLLER_HPP
#define AC_CONTROLLER_HPP

#include <memory>
#include <string>
#include <nav2_core/controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>


namespace ac_controller
{
    class ac_controller : public nav2_core::Controller
    {
        public:
        ac_controller() = default;
        ~ac_controller() override = default;
        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string,std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        geometry_msgs::msg::TwistStamped computeVelocityCmd(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity, nav2_core::GoalChecker* goal_checker) override;
        void setPlan(const nav_msgs::msg::Path& path) override;  //set the global plan
        void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

        protected:
        nav2_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);   //transform global plan into same frame as pose and clips poses

        bool transformPose(const std::shared_ptr<tf2_ros::Buffer tf,
                           const std::string frame,
                           const geometry_msgs::msg::PoseStamped& in_pose,
                           geometry_msgs::msg::PoseStamped& out_pose,
                           const rclcpp::Duration& transform_tolerance) const;
        
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;   
        rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
        rclcpp::Clock::SharedPtr clock_;

        double max_speed_;
        double lookahead_dist_;
        double wheelbase_;
        rclcpp::Duration transform_tolerance_{0, 0};

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;
    };
}

#endif