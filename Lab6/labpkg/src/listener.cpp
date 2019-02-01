#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "listener");
	ros::NodeHandle nh;
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener listener(buffer);
	geometry_msgs::TransformStamped transformStamped;

	while (nh.ok()){
		
		try{

			transformStamped = buffer.lookupTransform("husky_alpha/base_link", "husky_beta/base_link", ros::Time(0));
		
		}

		catch (tf2::TransformException &ex) {
		
			ROS_WARN("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		
		}

		ROS_INFO_STREAM("Obtained transformation " << transformStamped);

	}

	return 0;
}
