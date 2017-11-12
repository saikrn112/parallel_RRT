#ifndef FUSION_HPP
#define FUSION_HPP

/* Contains ROS nodehandlers and other essential methods*/
#include <ros/ros.h>

/* ROS message header files */
#include <rrt/rrt.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


/* RRT datastructures */
#include <rrt/Tree.hpp>

/* For getting and parsing parameters from parameter yaml files */
#include <XmlRpcException.h>

/* Armadillo Linear Algebra Library [refer: http://arma.sourceforge.net/docs.html ]*/
#include <Eigen/Dense>

#include <boost/bind.hpp>


#include <dynamic_reconfigure/server.h>


class RRT {
private:
  //members
public:
  bool nearestNeighbours();
  bool collisionChecker(); // need GPU implementation
  bool grow();
  bool getPath();
  int cellCost();
  int pathCost();




}
