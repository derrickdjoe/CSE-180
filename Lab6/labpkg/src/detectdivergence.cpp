#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <cmath>

double aTheta;
double eTheta;
bool stopGoal = false;

void getATheta(const sensor_msgs::Imu &msg){

	double roll, pitch, yaw;

	tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	aTheta = yaw * (180 / M_PI);

}

void getETheta(const geometry_msgs::PoseWithCovarianceStamped &msg){

	double roll, pitch, yaw;

	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
	
	eTheta = yaw * (180 / M_PI);

}

int main(int argc, char **argv){

	ros::init(argc, argv, "detectdivergence");
	ros::NodeHandle nh;

	ros::Subscriber subActual = nh.subscribe("/imu/data", 1000, &getATheta);
	ros::Subscriber subEstimate = nh.subscribe("/amcl_pose", 1000, &getETheta);

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	while(nh.ok()){

		ros::spinOnce();

		if(stopGoal){

			ac.cancelAllGoals();

		}

		if(abs(eTheta - aTheta) >= 10.0 && !stopGoal){

			ROS_INFO_STREAM("Stopping Goal");
			stopGoal = true;

		}

	}

}
