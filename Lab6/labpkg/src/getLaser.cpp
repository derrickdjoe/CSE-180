#include <ros/ros.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>

void laserMessageReceived(const sensor_msgs::LaserScan&msg){
	std::stringstream ss;

	for(int i = 0; i < msg.ranges.size(); i++){
		ss << msg.ranges[i] << std::endl;
		ROS_INFO_STREAM(msg.ranges[i] << std::endl);
	}
}

int main(int argc, char ** argv){
	ros::init(argc, argv, "getlaser");
	ros::NodeHandle nh;
	ros::Subscriber subLaster = nh.subscribe("/scan", 100, &laserMessageReceived);
	ros::spin();
}
