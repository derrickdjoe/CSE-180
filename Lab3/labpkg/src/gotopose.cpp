#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <valarray>

geometry_msgs::Twist goTo;
geometry_msgs::Twist routeTo;
geometry_msgs::TransformStamped rightHere;
geometry_msgs::Pose2D transfer;

float currentX, currentY, currentRotation = 0;
float goToX, goToY, goToZ;
double roll, pitch, yaw;
bool reachedTargetAngle = true;
bool reachedTargetCoordinates = true;
bool reachedTarget = true;

void gotloc(const tf::tfMessage &msg){

	if(msg.transforms[0].header.frame_id == "odom" && msg.transforms[0].child_frame_id == "base_link"){

		currentX = msg.transforms[0].transform.translation.x;
		currentY = msg.transforms[0].transform.translation.y;

		rightHere.transform.rotation.x = msg.transforms[0].transform.rotation.x;
		rightHere.transform.rotation.y = msg.transforms[0].transform.rotation.y;
		rightHere.transform.rotation.z = msg.transforms[0].transform.rotation.z;
		rightHere.transform.rotation.w = msg.transforms[0].transform.rotation.w;
		rightHere.transform.translation.x = msg.transforms[0].transform.translation.x;
		rightHere.transform.translation.y = msg.transforms[0].transform.translation.y;

		tf2::Quaternion q(rightHere.transform.rotation.x, rightHere.transform.rotation.y, rightHere.transform.rotation.z, rightHere.transform.rotation.w);
		tf2::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		
		ROS_INFO_STREAM("Current Location : X = " << currentX << " Y = " << currentY << "\n");
	}
}

void gotpose(const geometry_msgs::Pose2D &msg){

	if(reachedTarget){
		transfer = msg;
		goToX = transfer.x;
		goToY = transfer.y;
		goToZ = transfer.theta;
		reachedTargetAngle = false;
		reachedTargetCoordinates = false;
		reachedTarget = false;
		ROS_INFO_STREAM("Going to : X = " << goToX << " Y = " << goToY << " Theta = " << goToZ << "\n");
	}
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "gotopose");
	ros::NodeHandle nh;
	
	ros::Subscriber subtiwst = nh.subscribe("targetpose", 1000, &gotpose);
	ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);
	ros::Subscriber subtf = nh.subscribe("/tf", 1000, &gotloc);
	
	ros::Rate rate(1);


	while(ros::ok()){
		

			//for whatever reason, spinOnce does not do what it is supposed to do and only updates the current location once, so it will never know where it is after it checks the first location
			ros::spinOnce();

			int xToGo = goToX - currentX;
			int yToGo = goToY - currentY;

			float newAngle = atan2 (yToGo, xToGo);

			//finds angle before going forward

			while(reachedTargetAngle == false){

				routeTo.linear.x = 0;
				routeTo.angular.z = 0.087;
				pubtwist.publish(routeTo);
				ROS_INFO_STREAM(routeTo);
				rate.sleep();

					if( float(abs(goToZ - yaw)) < 0.175){
					
						reachedTargetAngle = true;
					}
			}

			if(reachedTargetAngle == true){

			//once angle is set, move forward until you reach the coordinates you want

				while(reachedTargetCoordinates == false){
				
					routeTo.linear.x = 0.2;
					routeTo.angular.z = 0;
					pubtwist.publish(routeTo);
					ROS_INFO_STREAM(routeTo);
					rate.sleep();

					if( double(abs(currentX - goToX)) <= 0.2 && double(abs(currentY - goToY)) <= 0.2){
						
						reachedTargetCoordinates = true;
						reachedTarget = true;
					}
				}
			}
			
			ros::spinOnce();


			/*ros::spinOnce();

			int xToGo = goToX - currentX;
			int yToGo = goToY - currentY;

			float newAngle = atan2(yToGo, xToGo);

			while(reachedTargetCoordinates == false){

				routeTo.linear.x = goToX;
				routeTo.linear.y = goToY;
				pubtwist.publish(routeTo);
				ROS_INFO_STREAM(routeTo);
				rate.sleep();

				if( double(abs(currentX - goToX)) <= 0.2 && double(abs(currentY - goToY)) <= 0.2){
				
						reachedTargetCoordinates = true;
				}
			}

			if(reachedTargetCoordinates = true){
				
				while(reachedTargetAngle == false){

					routeTo.linear.x = 0;
					routeTo.angular.z = 0.087;
					pubtwist.publish(routeTo);
					ROS_INFO_STREAM(routeTo);
					ROS_INFO_STREAM("Trying to get to : " << goToZ << "\n");
					ROS_INFO_STREAM("Current Yaw : " << yaw << "\n");
					ROS_INFO_STREAM("To Go : " << float(abs(goToZ - yaw)) << "\n");
					rate.sleep();

					if( float(abs(goToZ - yaw)) < 0.175){
					
						reachedTargetAngle = true;
					}
				}
			}*/
	}

}
