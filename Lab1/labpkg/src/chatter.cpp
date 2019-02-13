#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

int main(int argc,char **argv) {
  
  ros::init(argc,argv,"chatter");
  ros::NodeHandle nh;

  ros::Publisher pubint = nh.advertise<std_msgs::Int32>("integers",1000);
  ros::Publisher pubs = nh.advertise<std_msgs::String>("strings",1000);
  
  ros::Rate rate(2);
  int value=0;
  std_msgs::Int32 intToSend;
  std_msgs::String stringToSend;
  
  while (ros::ok()) {

    intToSend.data = value++;
    pubint.publish(intToSend);
    stringToSend.data = "CSE180-Robotics";
    pubs.publish(stringToSend);

    rate.sleep();
  }

}
