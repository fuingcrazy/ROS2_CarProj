#ifndef HYBRID_ASTAR_PLANNER_NODE_H_
#define HYBRID_ASTAR_PLANNER_NODE_H_


#include <cmath>
#include <vector>
#include <queue>
#include <iostream>
#include <string>
#include <algorithm>
#include <memory>

namespace hybrid_astar_planner{

struct Point {
   int x;
   int y;
   double theta;
   Point(double a,double b,double theta):x(a),y(b),theta(theta){}
};

struct Node;
using NodePtr = std::shared_ptr<Node>;
using NodeVector = std::vector<NodePtr>;

struct Node {
    Point pos;
    double g_cost;
    double h_cost;
    bool o;    //indicate if in open list
    bool c;
    double f_cost(void) const  {return g_cost+h_cost;}    //total cost, f = g + h
    int steering_grade_ = 0;          //shows steering angle
    NodePtr parent;     
    enum DIRECTION{forward = 0, backward = 1 ,other = 2 };      //set direction enumerate list
    Node(int a, int b,double theta) : g_cost(1e10),h_cost(1e10),parent(nullptr),pos(a,b,theta),o(false),c(false),direction_(forward) {}

    bool operator==(const Node& rhs){return pos.x==rhs.pos.x && pos.y == rhs.pos.y && abs(pos.theta - rhs.pos.theta) < 0.01;}
    void setOpen(){o = true;c = false;}
    void setClose(){o = false;c = true;}

    bool isClose(){return c;}
    bool isOpen(){return o;}
    
    DIRECTION direction_{};
  };


struct comparator{
    bool operator() (const NodePtr& n1, const NodePtr& n2)
    {
        if (n1->f_cost() == n2->f_cost())
        {
            return n1->h_cost > n2->h_cost;
        }
        else
            return (n1->f_cost() > n2->f_cost());
    }

};
using NodeList = std::priority_queue<NodePtr,NodeVector,comparator> ;

struct PriorityQueue {
    NodeList Q;
    NodeVector elements;

    bool empty()
    {
        return Q.empty();
    }

    bool is_member(const NodePtr& n)
    {
        for (auto &&node : elements)
        {
            if (*n == *node) return true;
        }
        return false;
    }

    NodePtr get_member(const NodePtr& n)
    {
        for (auto &&node : elements)
        {
            if (*n == *node) return node;
        }
        return NULL;
    }

    NodePtr top()
    {
        auto n = Q.top();
        Q.pop();
        int idx = -1;
        // Remove the node from the elements vector
        for (unsigned i = 0; i < elements.size(); i++)
        {
            if (*elements[i] == *n)
            {
                idx = i;
                break;
            }
        }
        elements.erase(elements.begin() + idx);
        return n;

    }

    void push(const NodePtr& n)
    {
        Q.push(n);
        elements.push_back(n);
    }
    int size()
    {
        return Q.size();
    }
    
};

}


#endif