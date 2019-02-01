#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <cstdlib>
#include <cmath>

geometry_msgs::Pose2D targetPose;
bool newPose = true;
float object;

void laserMessageReceived(const sensor_msgs::LaserScan &msg){

	float min = msg.ranges[0];

	for(int i = 1; i < msg.ranges.size(); i++){

		if(msg.ranges[i] < min){

			min = msg.ranges[i];

		}

	}

	object = min;

}

void randomPose(){

	targetPose.x = ((double)(rand() % 5 - 2))/5;
	targetPose.y = ((double)(rand() % 5 - 2))/5;
	targetPose.theta = ((double)(rand() % 5 - 2))/5;
	newPose = true;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "saferandomwalk");
	ros::NodeHandle nh;

	ros::Subscriber subLaster = nh.subscribe("/scan", 1000, &laserMessageReceived);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	ROS_INFO_STREAM("Waiting for action server...");

	while(!ac.waitForServer()){

	}

	ROS_INFO_STREAM("Connected to action server!");

	srand(time(NULL));

	randomPose();

	while(nh.ok()){

		ros::spinOnce();

		if(newPose){

			move_base_msgs::MoveBaseGoal goal;

			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = targetPose.x;
			goal.target_pose.pose.position.y = targetPose.y;
			goal.target_pose.pose.orientation.w = targetPose.theta;

			newPose = false;

			ac.sendGoal(goal);

		}

		if(object <= 0.2 || ac.getState().isDone()){

			ac.cancelGoal();
			randomPose();

		}

	}

}
