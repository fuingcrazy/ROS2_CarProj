#ifndef HYBRID_ASTAR_PLANNER_HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_HYBRID_ASTAR_PLANNER_H

#include <vector>
#include <string>
#include <memory>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include "rs_path.h"
#include "node.h"

namespace hybrid_astar_planner
{
    class hybrid_planner : public nav2_core::GlobalPlanner
    {
    public:
        hybrid_planner() = default;
        ~hybrid_planner() = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal) override;

    private:
        void DynamicModel(const double& stepsize, const double& phi, double &x, double &y, double& theta) const;
        double Mod2Pi(const double x) const;
        void getNeighbors(const NodePtr& cur_node, NodeVector& neighbors);
        bool check_arrive(const NodePtr& node, const NodePtr& goal);
        double setH(const NodePtr& cur_node, const NodePtr& terminal_node);
        double setG(const NodePtr& cur_node, const NodePtr& neighbor);
        nav_msgs::msg::Path extract_path(const NodePtr& end, const NodePtr& start);
        void SetCarShape();
        bool checkValid(double& wx,double& wy,double theta,unsigned int& mx,unsigned int& my);

        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Logger logger_{rclcpp::get_logger("HYBRID_ASTAR_PLANNER")};
        std::string name_,global_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_costmap_2d::Costmap2D* costmap_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_util::LifecycleNode::SharedPtr node;

  // Global Costmap
  std::shared_ptr<RSPath> rs_path_ptr_;           //path pointer

  // The global frame of the costmap

  inline double squared_distance(
    const NodePtr& n1,
    const NodePtr& n2)
  {
    double dx = n1->pos.x - n2->pos.x;
    double dy = n1->pos.y - n2->pos.y;
    return dx * dx + dy * dy;
  } 
  
  float minDistanceToObs_;       //to detect if goal point is valid
  float tolerance_;     //relax
  int maxIters_;
  float maxTurnRadius_;
  float turnPenalty_,noneStraight_,reversePenalty_,maxPlanningTime_,steering_change_penalty_;
  double carLength_,carWidth_,rear_axle_dist,wheel_base_,diagonal_, gamma_;

  double steering_radian_step_size_;
  double move_step_size_;
  double tie_breaker_;

  double shot_distance_;
  double segment_length_;
  int segment_length_discrete_num_;
  int steering_discrete_num_;    //possible steering states for path expansion 

  double path_length_ = 0.0;
  double angle_quantizations_ = 40.0;   //angle quantization for steering angle
  
  VecXd vehicle_shape_;
  MatXd vehicle_shape_discrete_;
  
    };
}

#endif
