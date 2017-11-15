#include <ros/ros.h>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include <stdlib.h>
#include <time.h>

using namespace ros ;

int main(int argc, char *argv[])
{
	init(argc,argv,"randomMapNode") ;

	NodeHandle T ;

	Publisher OCPublisher = T.advertise<nav_msgs::OccupancyGrid>("world",10000) ;
	Rate loop_rate(10) ;

	nav_msgs::OccupancyGrid map;

	map.info.resolution = 1.0;         // float32
	map.info.width      = atoi(argv[1]);           // uint32
	map.info.height     = atoi(argv[1]);           // uint32
	map.info.origin.position.x = 0 ;
	map.info.origin.position.y = 0 ;
	map.info.origin.position.z = 0 ;

	int size = map.info.height * map.info.width;

	std::vector<signed char> a ;

	for(int i = 0 ;i < size; i++)
	{
		a.push_back(0) ;
	}

	srand(time(NULL));

	for(int i = 0; i < size/atoi(argv[2]); i++)
	{
		a[rand() % size] = 100;
	}

	map.data = a;

	while(ok())
	{
		OCPublisher.publish(map) ;
		loop_rate.sleep() ;
	}
	
	return 0;
}

