#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "node.hpp"
#include "astar_planner.hpp"
#include "rs_path.h"

namespace hybrid_astar_planner
{
  std::vector<Node*> HybridAstar(Node* start,Node* goal,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);
  
  std::vector<Node*> dubinsShot(Node& start,const Node& goal)     //calculate dubins path from start to goal

  void updateH()

}