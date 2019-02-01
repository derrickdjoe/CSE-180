#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc,char **argv) {
  
  ros::init(argc,argv,"sendVel");
  ros::NodeHandle nh;

  ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);


  geometry_msgs::Twist toSend;
  ros::Rate rate(3);
  int count = 0;
  bool turn = true;

  while(ros::ok()){
	
	if(turn == false){
		toSend.linear.x = 500;
		toSend.angular.z = 0;
		pubtwist.publish(toSend);
		turn = true;
		rate.sleep();
	}else{
		toSend.angular.z = 0.90;
		toSend.linear.x = 0;
		pubtwist.publish(toSend);
		turn = false;
		rate.sleep();
	}

	//pubtwist.publish(toSend);
	ROS_INFO_STREAM(toSend);
	//rate.sleep();

  }

}
