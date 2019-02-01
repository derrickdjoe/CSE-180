#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/impl/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
		
	ros::init(argc, argv, "repeat");
	ros::NodeHandle nh;
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener(buffer);
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::TransformStamped transformStampedLast;
	tf2::Transform transformNow;
	tf2::Transform transformLast;
	tf2::Transform transformDiff;
	geometry_msgs::Twist toSend;
	int currentX, currentY, currentZ;

	ros::Publisher pubtwist = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 1000);
		

	while(nh.ok()){

		float xLast, xNew, yLast, yNew;
		double lastRoll, lastPitch, lastYaw, newRoll, newPitch, newYaw;

		try{

			transformStampedLast = buffer.lookupTransform("husky_alpha/base_link", "husky_beta/base_link", ros::Time(0));
			//tf2::convert(transformStampedLast.transform, transformLast);
	
		}

		catch(tf2::TransformException &ex){

			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
	
		}

		xLast = transformStampedLast.transform.translation.x;
		yLast = transformStampedLast.transform.translation.y;

		tf2::Quaternion q(transformStampedLast.transform.translation.x, transformStampedLast.transform.translation.y, transformStampedLast.transform.translation.z, transformStampedLast.transform.rotation.w);

		tf2::Matrix3x3 m(q);
		m.getRPY(lastRoll, lastPitch, lastYaw);
		

		try{
		
			transformStamped = buffer.lookupTransform("husky_alpha/base_link", "husky_beta/base_link", ros::Time(0));
			//tf2::convert(transformStamped.transform, transformNow);
			//transformDiff = transformNow.inverseTimes(transformLast);

			//currentX = transformDiff.transform.translation.x
			//currentY = transformDiff.transform.translation.y;

			//tf2::convert(transformDiff, toSend);
			
			//transformStampedLast = transformStamped;

		}

		catch(tf2::TransformException &ex){

			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		
		}

		xNew = transformStamped.transform.translation.x;
		yNew = transformStamped.transform.translation.y;
		
		tf2::Quaternion q1(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z, transformStamped.transform.rotation.w);

		tf2::Matrix3x3 m1(q1);
		m1.getRPY(newRoll, newPitch, newYaw);

		//Calc distance and angular change
		toSend.linear.x = sqrt( ((xNew - xLast) * (xNew - xLast)) + ((yNew - yLast) * (yNew - yLast)) );
		toSend.angular.z = (newYaw - lastYaw);

		pubtwist.publish(toSend);

		

	}

	return 0;
}
