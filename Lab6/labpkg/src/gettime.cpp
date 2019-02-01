#include <ros/ros.h>
#include <std_msgs/Time.h>

void timeMessageReceived(const std_msgs::Time&msg) {
	int secs = msg.data.toSec();

	int seconds = secs % 60;
	int minutes = (secs % 3600) / 60;
	int hour = (secs % 216000) / 3600;

	ROS_INFO_STREAM("Received time (UTC), Hours: " << hour << " Minutes: " << minutes << " Seconds: " << seconds);
}

int main(int argc,char ** argv) {

  ros::init(argc,argv,"listener");
  ros::NodeHandle nh;

	ros::Subscriber subTime = 
	nh.subscribe("timetopic",1000,&timeMessageReceived);

  ros::spin();
}
