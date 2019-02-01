#include <ros/ros.h>
#include <std_msgs/Time.h>

int main(int argc,char **argv) {
  
  ros::init(argc,argv,"sendtime");
  ros::NodeHandle nh;

  ros::Publisher pubint = nh.advertise<std_msgs::Time>("timetopic",1000);
  
  ros::Rate rate(1);

	std_msgs::Time currentTime;

  while (ros::ok()) {

		currentTime.data = ros::Time::now();
    pubint.publish(currentTime);
	ROS_INFO_STREAM("Sending " << currentTime);
    rate.sleep();
  }

}
