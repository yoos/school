/**
 * @file cb_vision.cpp
 */

#include <ros/ros.h>

#include <cb_vision/cb_puck_finder.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cb_vision");
	ros::NodeHandle nh;

	CBPuckFinder ic(nh);

	ros::spin();

	return 0;
}

