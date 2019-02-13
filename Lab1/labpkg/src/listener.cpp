#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

void intMessageReceived(const std_msgs::Int32&msg) {
  ROS_INFO_STREAM("Received integer " << msg.data);
}

void stringMessageReceived(const std_msgs::String&msg) {
  ROS_INFO_STREAM("Received string " << msg.data);
}

int main(int argc,char ** argv) {

  ros::init(argc,argv,"listener");
  ros::NodeHandle nh;

  ros::Subscriber subInt =
    nh.subscribe("integers",1000,&intMessageReceived);
  ros::Subscriber subString =
    nh.subscribe("strings",1000,&stringMessageReceived); 

  ros::spin();
}
