#include <rrt/rrt.hpp>

using namespace std;


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, bool* mapLoaded, nav_msgs::OccupancyGrid* m)
{
	*mapLoaded = true;
	*m = *msg;
	ROS_INFO_STREAM("map loaded");
}

void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, bool* startLoaded, geometry_msgs::PoseWithCovarianceStamped* start)
{
	*startLoaded = true;
	*start = *msg;
	ROS_INFO_STREAM("start loaded");

}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, bool* goalLoaded, geometry_msgs::PoseStamped* goal)
{
	*goalLoaded = true;
	*goal = *msg;
	ROS_INFO_STREAM("goal loaded");

}

int findIndex(geometry_msgs::Pose pose, nav_msgs::OccupancyGrid map)
{
	float relX = pose.position.x - map.info.origin.position.x;
	float relY = pose.position.y - map.info.origin.position.y;

	// ROS_INFO_STREAM(relX);
	// ROS_INFO_STREAM(relY);

	relX = (int)(relX/map.info.resolution);
	relY = (int)(relY/map.info.resolution);

	// ROS_INFO_STREAM(relX);
	// ROS_INFO_STREAM(relY);

	return int((int)relY * map.info.width + (int)relX);
}


int main(int argc, char ** argv){
  init(argc, argv, "astarNode");
  NodeHandle nh;

  //
  Graph costGraph;
  //
  vector<Cell*> finalPath;

  bool mapLoaded = false, startLoaded = false, goalLoaded = false;

  Publisher pub = nh.advertise<nav_msgs::Path>("astarPath", 1000);

  nav_msgs::OccupancyGrid map;
  geometry_msgs::PoseWithCovarianceStamped start;
  geometry_msgs::PoseStamped goal;

  Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, boost::bind(mapCallback, _1, &mapLoaded, &map));
  Subscriber startSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000, boost::bind(startCallback, _1, &startLoaded, &start));
  Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000, boost::bind(goalCallback, _1, &goalLoaded, &goal));

  Rate loopRate(10);

  while(!(mapLoaded && startLoaded && goalLoaded) && ok())
  {
    spinOnce();
  }

  ROS_INFO_STREAM(map.info.width);
  ROS_INFO_STREAM(map.info.height);
  ROS_INFO_STREAM(map.info.origin.position.x);
  ROS_INFO_STREAM(map.info.origin.position.y);

  // ROS_INFO_STREAM(findIndex(start.pose.pose, map) % map.info.width);
  // ROS_INFO_STREAM(findIndex(start.pose.pose, map) / map.info.width);
  // ROS_INFO_STREAM(findIndex(goal.pose, map) % map.info.width);
  // ROS_INFO_STREAM(findIndex(goal.pose, map) / map.info.width);

  //
  costGraph.setNodes(map);

  finalPath = costGraph.findPath(findIndex(start.pose.pose, map), findIndex(goal.pose, map));

  nav_msgs::Path p;
  std_msgs::Header h;

  h.frame_id = "odom";

  vector<geometry_msgs::PoseStamped> poses;

  for(auto it = finalPath.begin(); it != finalPath.end(); it++)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = ((*it)->x) * map.info.resolution + map.info.origin.position.x;
    pose.pose.position.y = ((*it)->y) * map.info.resolution + map.info.origin.position.y;

    poses.push_back(pose);
  }

  p.header = h;
  p.poses = poses;

  ROS_INFO_STREAM("done");

  while(ok())
  {
    pub.publish(p);
    loopRate.sleep();
  }

  return 0;

}
