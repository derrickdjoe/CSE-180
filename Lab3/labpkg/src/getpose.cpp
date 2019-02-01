#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>

int main(int argc, char **argv){

	ros::init(argc, argv, "getpose");
	ros::NodeHandle nh;

	ros::Publisher pubpose = nh.advertise<geometry_msgs::Pose2D>("targetpose", 1000);
	ros::Rate rate(1);

	geometry_msgs::Pose2D sendData;
	int x, y, theta = 0;

	while(ros::ok()){
		
		std::cout << "Enter a X" << std::endl;
		std::cin >> sendData.x;
		std::cout << "Enter a Y" << std::endl;
		std::cin >> sendData.y;
		std::cout << "Enter a Theta" << std::endl;
		std::cin >> sendData.theta;
		
		pubpose.publish(sendData);
		ROS_INFO_STREAM(sendData);
		rate.sleep();
	}
}
