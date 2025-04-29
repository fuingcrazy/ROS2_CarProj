#include "hybrid_astar_planner/hybrid_astar_planner.h"
#include "hybrid_astar_planner/node.h"
#include "hybrid_astar_planner/dubins.h"
#include "hybrid_astar_planner/timer.h"
#include <assert.h>
#include <iostream>

namespace hybrid_astar_planner
{

 void hybrid_planner::configure(
     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent;
    auto node = node_.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
  
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".tolerance", rclcpp::ParameterValue(
      0.1));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".minDistanceToObs_", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".carWidth", rclcpp::ParameterValue(
      0.3));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".carLength", rclcpp::ParameterValue(
      0.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".wheelBase", rclcpp::ParameterValue(
      0.45));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".turnPenalty", rclcpp::ParameterValue(
      2.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".steeringChangePenalty", rclcpp::ParameterValue(
      1.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".noneStraight", rclcpp::ParameterValue(
      1.2));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".reversePenalty", rclcpp::ParameterValue(
      4.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".maxIters", rclcpp::ParameterValue(
      6000));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".maxTurnRadius", rclcpp::ParameterValue(
      1.5));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".maxPlanningTime", rclcpp::ParameterValue(
      5.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".rearAxleDist", rclcpp::ParameterValue(
      1.0));    //length from front bumper to rear wheels
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".segmentLength", rclcpp::ParameterValue(
      1.0));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".moveStepSize", rclcpp::ParameterValue(
      0.2));
  

  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".minDistanceToObs_", minDistanceToObs_);
  node->get_parameter(name_ + ".carWidth", carWidth_);
  node->get_parameter(name_ + ".carLength", carLength_);
  node->get_parameter(name_ + ".wheelBase", wheel_base_);       //this is distance between front and real axles
  node->get_parameter(name_ + ".rearAxleDist", rear_axle_dist);
  node->get_parameter(name_ + ".turnPenalty", turnPenalty_);
  node->get_parameter(name_ + ".steeringChangePenalty", steering_change_penalty_);
  node->get_parameter(name_ + ".noneStraight", noneStraight_);
  node->get_parameter(name_ + ".reversePenalty", reversePenalty_);
  node->get_parameter(name_ + ".maxIters", maxIters_);
  node->get_parameter(name_ + ".maxTurnRadius", maxTurnRadius_);
  node->get_parameter(name_ + ".maxPlanningTime", maxPlanningTime_);
  node->get_parameter(name_ + ".segmentLength", segment_length_);
  node->get_parameter(name_ + ".moveStepSize", move_step_size_);
  
  
  steering_radian_step_size_ = maxTurnRadius_ / angle_quantizations_;    //minimal steering angle unit
  segment_length_discrete_num_ = static_cast<int>(segment_length_ / move_step_size_);
  rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(maxTurnRadius_));       //initialize rs path by giving turning
                                                                                         //radius 
  tie_breaker_ = 1.0 + 1e-3;
  SetCarShape();
  RCLCPP_INFO(logger_, "Hybrid Astar planner initialized");
}

void hybrid_planner::cleanup()
{
   RCLCPP_INFO(
       logger_, "CleaningUp plugin %s of type Astar",
       name_.c_str());
}

void hybrid_planner::activate()
{
   RCLCPP_INFO(
       logger_, "Activating plugin %s of type Hybrid Astar",
       name_.c_str());
}

void hybrid_planner::deactivate()
{
   RCLCPP_INFO(
       logger_, "Deactivating plugin %s of type Hybrid Astar",
       name_.c_str());
}

void hybrid_planner::SetCarShape()
{
 vehicle_shape_.resize(8);
 vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, carWidth_ / 2);
 vehicle_shape_.block<2, 1>(2, 0) = Vec2d(carLength_ - rear_axle_dist, carWidth_ / 2);
 vehicle_shape_.block<2, 1>(4, 0) = Vec2d(carLength_ - rear_axle_dist, -carWidth_ / 2);
 vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -carWidth_ / 2);

 const double step_size = move_step_size_;
 const auto N_length = static_cast<unsigned int>(carLength_ / step_size);
 const auto N_width = static_cast<unsigned int> (carWidth_ / step_size);
 vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

 const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                  - vehicle_shape_.block<2, 1>(0, 0)).normalized();
 for (unsigned int i = 0; i < N_length; ++i) {
     vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
             = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
     vehicle_shape_discrete_.block<2, 1>(0, i)
             = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
 }

 const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                  - vehicle_shape_.block<2, 1>(2, 0)).normalized();
 for (unsigned int i = 0; i < N_width; ++i) {
     vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
             = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
     vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
             = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
 }
 diagonal_ = std::sqrt(carLength_ * carLength_ + carWidth_ * carWidth_) / 2.0;
 gamma_ = std::atan2(carWidth_,carLength_);
}

void hybrid_planner::DynamicModel(const double& stepsize, const double& phi, double &x, double &y, double& theta) const
{
   x = x + stepsize * std::cos(theta);
   y = y + stepsize * std::sin(theta);
   theta = Mod2Pi(theta + stepsize / wheel_base_ * std::tan(phi));
}

double hybrid_planner::Mod2Pi(const double x) const
{
   double v = fmod(x, 2 * M_PI);
   if (v < -M_PI) 
       v += 2.0 * M_PI;
   else if (v > M_PI) 
       v -= 2.0 * M_PI;
   return v;
}

 void hybrid_planner::getNeighbors(const NodePtr& cur_node, NodeVector neighbors)
 {
   neighbors.clear();
   bool has_obstacle = false;
   double wx,wy,theta;
   unsigned int mx,my;
   costmap_->mapToWorld(cur_node->pos.x,cur_node->pos.y,wx,wy);
   theta = cur_node->pos.theta;
   RCLCPP_INFO(logger_," word pos:(%d,%d)",mx,my);
   //forward
   for(int i = -angle_quantizations_;i<=angle_quantizations_;i++){

     has_obstacle = false;
     const double phi = i * steering_radian_step_size_;     //potential steering angle
     for(int j = 0;j < segment_length_discrete_num_;j++){
       DynamicModel(move_step_size_,phi,wx,wy,theta);           //apply dynamic model to get the next state
       if(!costmap_->worldToMap(wx,wy,mx,my) || !checkValid(wx,wy,theta,mx,my)) {
           has_obstacle = true;
           break;
       }
     }
     if(checkValid(wx,wy,theta,mx,my)){
       auto neighbor_ptr = std::make_shared<Node>(mx,my,theta);
       neighbor_ptr->parent = cur_node;
       neighbor_ptr->direction_ = Node::forward;
       neighbor_ptr->steering_grade_ = i;
       neighbors.push_back(neighbor_ptr);
     }
    
     //backward
     has_obstacle = false;
     theta = cur_node->pos.theta;
     costmap_->mapToWorld(cur_node->pos.x,cur_node->pos.y,wx,wy);     //set the state back to the current node

     for(int j = 0;j < segment_length_discrete_num_;j++){
       DynamicModel(-move_step_size_,phi,wx,wy,theta);           //apply dynamic model to get the next state
       if(!costmap_->worldToMap(wx,wy,mx,my) || !checkValid(wx,wy,theta,mx,my)) {
           has_obstacle = true;
           break;
       }   //push intermediate points

  }  
  if(checkValid(wx,wy,theta,mx,my)){
    auto neighbor_ptr = std::make_shared<Node>(mx,my,theta);
    neighbor_ptr->parent = cur_node;
    neighbor_ptr->direction_ = Node::backward;
    neighbor_ptr->steering_grade_ = i;
    neighbors.push_back(neighbor_ptr);
  } 
  }
}

double hybrid_planner::setH(const NodePtr& cur_node, const NodePtr& terminal_node)
{
  double h = std::sqrt(squared_distance(cur_node, terminal_node));

  if(h < 3.0 * shot_distance_){
   h = rs_path_ptr_->Distance(cur_node->pos.x,cur_node->pos.y,cur_node->pos.theta,terminal_node->pos.x,terminal_node->pos.y,terminal_node->pos.theta);
  }
  return h;
}

double hybrid_planner::setG(const NodePtr& cur_node, const NodePtr& neighbor)
{
  double g;
  if(neighbor->direction_ == Node::forward){
     if(neighbor->steering_grade_ != cur_node->steering_grade_){
       g = neighbor->steering_grade_ == 0 ? segment_length_ * turnPenalty_ : segment_length_ * turnPenalty_ * steering_change_penalty_;
     }
     else
       g = neighbor->steering_grade_ == 0 ? segment_length_ : segment_length_ * turnPenalty_;
  }
  else   //backward
  {
     if(neighbor->steering_grade_ != cur_node->steering_grade_){
       g = neighbor->steering_grade_ == 0 ? segment_length_ * turnPenalty_  : segment_length_ * turnPenalty_ * steering_change_penalty_;
     } 
     else
       g = neighbor->steering_grade_ == 0 ? segment_length_ : segment_length_ * turnPenalty_;
    g *= reversePenalty_;
  }
  return g;
}

nav_msgs::msg::Path hybrid_planner::createPlan(
   const geometry_msgs::msg::PoseStamped & start,
   const geometry_msgs::msg::PoseStamped & goal)
{
   nav_msgs::msg::Path global_path;

   /*This is a corner case of overlapping start and goal point*/
   if (std::abs(start.pose.position.x - goal.pose.position.x) < carLength_ &&
       std::abs(start.pose.position.y - goal.pose.position.y) < carWidth_) 
     {
      RCLCPP_INFO(logger_, "Start and goal are the same,doing nothing...");
       unsigned int mx, my;
       costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
       if (costmap_->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
         RCLCPP_WARN(logger_, "Failed to create a unique pose path because of obstacles");
         return global_path;
       }
       global_path.header.stamp = clock_->now();
       global_path.header.frame_id = global_frame_;
       geometry_msgs::msg::PoseStamped pose;
       pose.header = global_path.header;
       pose.pose.position.z = 0.0;
   
       pose.pose = start.pose;
       // if we have a different start and goal orientation, set the unique path pose to the goal
       // orientation, unless use_final_approach_orientation=true where we need it to be the start
       // orientation to avoid movement from the local planner
       global_path.poses.push_back(pose);
       return global_path;
     }
 unsigned int sx,sy,ex,ey;
 double theta_s = tf2::getYaw(start.pose.orientation);
 double theta_g = tf2::getYaw(goal.pose.orientation);

 // Create local variables for the coordinates
 RCLCPP_INFO(logger_,"Start theta: %.2f, goal theta: %.2f",theta_s,theta_g);
 double start_x = start.pose.position.x;
 double start_y = start.pose.position.y;
 double goal_x = goal.pose.position.x;
 double goal_y = goal.pose.position.y;

 if (!checkValid(start_x, start_y, theta_s, sx, sy) || 
     !checkValid(goal_x, goal_y, theta_g, ex, ey)){
    RCLCPP_WARN(logger_,"Start/End point not available!");
    return global_path;
 }
 auto start_node = std::make_shared<Node>(sx,sy,theta_s);
 auto end_node = std::make_shared<Node>(ex,ey,theta_g);
 start_node->setOpen();
 start_node->g_cost = 0.0;
 start_node->h_cost = setH(start_node,end_node);
 PriorityQueue Open;
 NodeVector neighbor_nodes;
 Open.push(start_node);

 int count = 0;
 RCLCPP_INFO(logger_,"Start Position: (%.2f,%.2f,%.2f)",start_x,start_y,theta_s);
 RCLCPP_INFO(logger_,"End Position: (%.2f,%.2f,%.2f)",goal_x,goal_y,theta_g);
 RCLCPP_INFO(logger_,"Start Grid: (%d,%d), end grid:(%d,%d)",sx,sy,ex,ey);
 RCLCPP_INFO(logger_,"Initialization finished, start planning...");
 double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0,check_collision_time;

 while(!Open.empty() && count < maxIters_){
   auto cur_ptr = Open.Q.top();        //select the node with the lowest f_cost
   RCLCPP_INFO(logger_,"Current node: (%d,%d,%.2f)",cur_ptr->pos.x,cur_ptr->pos.y,cur_ptr->pos.theta);
   cur_ptr->setOpen();;
   count++;
   Open.Q.pop();
   if(count > maxIters_ ){
     RCLCPP_WARN(logger_,"Failed to find a path, max iterations reached!");
     return global_path;
   }

   Timer get_neighbor_time;
   getNeighbors(cur_ptr,neighbor_nodes);
   RCLCPP_INFO(logger_,"Get %d neighbors",neighbor_nodes.size());
   neighbor_time += get_neighbor_time.End();
    
for(auto& neighbor_ptr : neighbor_nodes){
       Timer time_get_neighbor;
       const double current_h = setH(cur_ptr,neighbor_ptr) * tie_breaker_;
       compute_h_time = compute_h_time + time_get_neighbor.End();
       Timer time_get_g;
       const double neighbor_edge_cost = setG(cur_ptr,neighbor_ptr);
       compute_g_time = compute_g_time + time_get_g.End();
       
       if(check_arrive(neighbor_ptr,end_node)){
        RCLCPP_INFO(logger_,"Path length: %.2f",path_length_);
        RCLCPP_INFO(logger_,"Search neighbor time: %.2f",neighbor_time);
        RCLCPP_INFO(logger_,"Compute h time: %.2f",compute_h_time);
        RCLCPP_INFO(logger_,"Compute g time: %.2f",compute_g_time);
        RCLCPP_INFO(logger_,"Check collision time: %.2f",check_collision_time);
        global_path = extract_path(cur_ptr,start_node);
        return global_path;
       }
      
       if(!neighbor_ptr->isClose()){
          if(neighbor_ptr->isOpen()){
             double temp_g_cost = cur_ptr->g_cost + neighbor_edge_cost;
             if(temp_g_cost < neighbor_ptr->g_cost){   //prune the path
                 neighbor_ptr->g_cost = temp_g_cost;
                 neighbor_ptr->h_cost = current_h;
             }
          }
          else
            {
              neighbor_ptr->g_cost = cur_ptr->g_cost + neighbor_edge_cost;
              neighbor_ptr->h_cost = current_h;
              neighbor_ptr->setOpen();
              Open.Q.push(neighbor_ptr);       //push the node to open list
            }
       }
       else
         continue;       //delete the node in close list
}
cur_ptr->setClose();
RCLCPP_INFO(logger_," Openlist size: %d",Open.size());
}
}

bool hybrid_planner::checkValid(double& wx, double& wy, double theta, unsigned int& mx, unsigned int& my)   //check if vehicle is within the map and not in collision
{
    std::pair<double,double> fl,fr,rl,rr;      //4 corners of vehicle
    unsigned int flx,fly,frx,fry,rlx,rly,rrx,rry;
    if(!costmap_->worldToMap(wx,wy,mx,my)) return false;
    fl.first = wx + std::sin(theta - gamma_) * diagonal_;
    fl.second = wy + std::cos(theta - gamma_) * diagonal_;
    fr.first = wx + std::sin(theta + gamma_) * diagonal_;
    fr.second = wy + std::cos(theta + gamma_) * diagonal_;
    rl.first = wx - std::sin(gamma_ + theta) * diagonal_;
    rl.second = wy - std::cos(gamma_ + theta) * diagonal_;
    rr.first = wx + std::sin(gamma_ - theta) * diagonal_;
    rr.second = wy - std::cos(gamma_ - theta) * diagonal_;
    //RCLCPP_INFO(logger_,"fl: (%.2f,%.2f), fr: (%.2f,%.2f), rl: (%.2f,%.2f), rr: (%.2f,%.2f)",fl.first,fl.second,fr.first,fr.second,rl.first,rl.second,rr.first,rr.second);
    costmap_->worldToMap(fl.first, fl.second, flx, fly);
    costmap_->worldToMap(fr.first, fr.second, frx, fry);
    costmap_->worldToMap(rl.first, rl.second, rlx, rly);
    costmap_->worldToMap(rr.first, rr.second, rrx, rry);          //transfer to map coordinates
    if (costmap_->getCost(flx, fly) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        costmap_->getCost(frx, fry) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        costmap_->getCost(rlx, rly) == nav2_costmap_2d::LETHAL_OBSTACLE ||
        costmap_->getCost(rrx, rry) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        return false;
    }
    return true;
}

nav_msgs::msg::Path hybrid_planner::extract_path(const NodePtr& end, const NodePtr& start)
{
    nav_msgs::msg::Path path;
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    NodePtr temp = end;
    while(temp != start) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        double mx,my;
        costmap_->mapToWorld(temp->pos.x,temp->pos.y,mx,my);
        pose.pose.position.x = mx;
        pose.pose.position.y = my;
        pose.pose.position.z = temp->pos.theta;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,1,temp->pos.theta));
        path.poses.push_back(pose);
        temp = temp->parent;
        path_length_ += segment_length_;
    }
    std::reverse(path.poses.begin(),path.poses.end());
    return path;
}

bool hybrid_planner::check_arrive(const NodePtr& node, const NodePtr& goal)
{
    if(squared_distance(node,goal) < tolerance_) {
        RCLCPP_INFO(logger_, "Goal arrived!");
        return true;
    }
    return false;
}

} // namespace hybrid_astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::hybrid_planner, nav2_core::GlobalPlanner)