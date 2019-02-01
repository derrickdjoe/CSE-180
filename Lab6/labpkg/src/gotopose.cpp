#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D targetPose;
bool newPose = true;

void gotPose(const geometry_msgs::Pose2D &msg){

	ROS_INFO_STREAM("Got target pose");
	targetPose.x = msg.x;
	targetPose.y = msg.y;
	targetPose.theta = msg.theta;
	newPose = true;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "gtopose");
	ros::NodeHandle nh;
	ros::Subscriber getPose = nh.subscribe("/targetpose", 1000, &gotPose);
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	
	ROS_INFO_STREAM("Waiting for server to be available...");

	while(!ac.waitForServer()){

	}

	ROS_INFO_STREAM("done!");

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

	}

}
