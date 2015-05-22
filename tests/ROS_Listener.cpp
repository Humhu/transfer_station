#include <ros/ros.h>

#include "transfer_station/Pose2DWithCovarianceStamped.h"

using namespace transfer_station;

void ROSCallback( const Pose2DWithCovarianceStamped::ConstPtr& msg )
{
	
	std::cout << "pose: " << msg->pose.pose.x << ", "
	<< msg->pose.pose.y << ", " << msg->pose.pose.theta << std::endl;
	
}

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "ros_listener" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	ros::Subscriber sub = nh.subscribe( "/chatter", 1, &ROSCallback );
	
	while( ros::ok() )
	{
		ros::spinOnce();
	}
}
