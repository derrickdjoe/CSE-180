#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Eigenvalues>
#include <cmath>

void gotPose(const nav_msgs::Odometry &msg){

	Eigen::Matrix3d coMatrix;
	int n = 0;
	
	for(int i = 0; i < 3; i++){

		for(int j = 0; j < 3; j++){

			while(n < 36){

				if(abs(msg.pose.covariance[n]) > 0.0001){

					coMatrix(i, j) = msg.pose.covariance[n];
					n++;
					break;

				}

			n++;

			}

		}

	}


	std::complex<double> v1 = coMatrix.eigenvalues()[0];
	std::complex<double> v2 = coMatrix.eigenvalues()[1];
	std::complex<double> v3 = coMatrix.eigenvalues()[2];

	double a = v1.real();
	double b = v2.real();
	double c = v3.real();

	ROS_INFO_STREAM("Volume: " << (4.0/3.0) * a * b * c * M_PI);

}

int main(int argc, char **argv){

	ros::init(argc, argv, "unvertainty");
	ros::NodeHandle nh;
	
	ros::Subscriber subPose = nh.subscribe("/odometry/filtered", 1000, &gotPose);

	ros::spin();

}
