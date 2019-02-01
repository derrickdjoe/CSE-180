#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>     /* srand, rand */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random");
  ros::NodeHandle n;

  double iSecret = 0;
  double jSecret = 0;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("husky_beta/husky_velocity_controller/cmd_vel", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::Twist msg;

  srand( time( NULL ) );

  int counter = 0;

  ros::Time startTime = ros::Time::now();
  ros::Duration duration = ros::Duration( rand() % 2 + 1 );

  int toggle = 1;

  while (ros::ok())
  {
       if ( ( ros::Time::now().sec - startTime.sec ) >= duration.sec )
  	{
	  	startTime = ros::Time::now();

      if (toggle)
      {
        duration = ros::Duration( rand() % 2 + 1 );
        toggle = 0;
        iSecret = 0;
        jSecret = ((double)(rand() % 5 - 2))/5;
      }
      else
      {
        duration = ros::Duration( rand() % 2 + 1 );
        toggle = 1;
        iSecret = ((double)(rand() % 5 - 2))/5;
        jSecret = 0;
      }
    }

       //iSecret /=5;
       //jSecret /= 5;
       
  	msg.linear.x = iSecret;
  	msg.angular.z = jSecret;

  	chatter_pub.publish(msg);
	ROS_INFO_STREAM("Publishing " << msg);
  	counter++;
  }

  return 0;
}
