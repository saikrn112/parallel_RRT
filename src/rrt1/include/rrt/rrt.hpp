#ifndef RRT_HPP
#define RRT_HPP

/* Contains ROS nodehandlers and other essential methods*/
#include <ros/ros.h>

/* ROS message header files */
#include <rrt/rrt.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


/* RRT datastructures */
#include <rrt/Tree.hpp>
#include <rrt/nanoflann.hpp>


/* For getting and parsing parameters from parameter yaml files */
#include <XmlRpcException.h>

/* Armadillo Linear Algebra Library [refer: http://arma.sourceforge.net/docs.html ]*/
#include <Eigen/Dense>

#include <boost/bind.hpp>


#include <dynamic_reconfigure/server.h>


namespace RRT{

  class RRTStar {
  private:
    //members
  public:
    RRTStar();
    ~RRTStar();
    bool sample();
    void extend();
    bool nearestNeighbours();
    bool collisionChecker(); // need GPU implementation
    bool grow();
    bool getPath(); // using kinematics
    int cellCost();
    int pathCost();
    vector<State*> path();



  };


}
