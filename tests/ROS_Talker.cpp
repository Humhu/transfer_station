#include <ros/ros.h>

#include "transfer_station/Pose2DWithCovarianceStamped.h"

using namespace transfer_station;

int main( int argc, char** argv )
{

	ros::init( argc, argv, "ros_talker" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	ros::Publisher pub = nh.advertise<Pose2DWithCovarianceStamped>( "/chatter", 1 );
	
	double loopRate;
	ph.param<double>( "rate", loopRate, 10.0 );
	
	ros::Rate rate( loopRate );
	unsigned int seq = 0;
	while( ros::ok() )
	{
		
		Pose2DWithCovarianceStamped msg;
		msg.header.seq = seq++;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "0";
		msg.pose.pose.x = 0.1*seq;
		msg.pose.pose.y = 0.2*seq;
		msg.pose.pose.theta = -0.1*seq;
		msg.pose.covariance[0] = 1;
		msg.pose.covariance[1] = 0;
		msg.pose.covariance[2] = 0;
		msg.pose.covariance[3] = 0;
		msg.pose.covariance[4] = 1;
		msg.pose.covariance[5] = 0;
		msg.pose.covariance[6] = 0;
		msg.pose.covariance[7] = 0;
		msg.pose.covariance[8] = 1;
		
		std::cout << "pose: " << msg.pose.pose.x << ", "
			<< msg.pose.pose.y << ", " << msg.pose.pose.theta << std::endl;
		
		pub.publish( msg );
		
		ros::spinOnce();
		rate.sleep();
		
	}
	
}
