#include <rrt/rrt.hpp>

using namespace std;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, bool* mapLoaded, nav_msgs::OccupancyGrid* m){
	*mapLoaded = true;
	*m = *msg;
	ROS_INFO_STREAM("map loaded");
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, bool* startLoaded, geometry_msgs::PoseWithCovarianceStamped* start){
	*startLoaded = true;
	*start = *msg;
	ROS_INFO_STREAM("start loaded");
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, bool* goalLoaded, geometry_msgs::PoseStamped* goal){
	*goalLoaded = true;
	*goal = *msg;
	ROS_INFO_STREAM("goal loaded");
}

int main(int argc, char ** argv){
  init(argc, argv, "RRT");
  NodeHandle nh;

  bool mapLoaded = false, startLoaded = false, goalLoaded = false;
  nav_msgs::OccupancyGrid map;
  geometry_msgs::PoseWithCovarianceStamped start;
  geometry_msgs::PoseStamped goal;

	Publisher pub = nh.advertise<nav_msgs::Path>("/path", 1000);
  Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, boost::bind(mapCallback, _1, &mapLoaded, &map));
  Subscriber startSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000, boost::bind(startCallback, _1, &startLoaded, &start));
  Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000, boost::bind(goalCallback, _1, &goalLoaded, &goal));

  Rate loopRate(10)
  loopRate.sleep();
  return 0;

}
