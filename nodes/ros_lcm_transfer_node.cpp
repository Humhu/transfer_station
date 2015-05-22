#include <ros/ros.h>

#include <lcm/lcm.h>
#include <sys/select.h>

#include "transfer_station/JointCommandL2R.h"

using namespace transfer_station;

void lcmSpinOnce(lcm::LCM *lcm);

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "ros_lcm_transfer_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");
	
	// 1. Create the global lcm interface
	lcm::LCM lcm;
	
	// 2. Set up all the routing objects
	std::vector< Router::Ptr > routers;
	
	if( !ph.hasParam( "routes" ) )
	{
		throw std::runtime_error( "No routes specified. Terminating." );
	}
	
	XmlRpc::XmlRpcValue routes;
	ph.getParam( "routes", routes );
	
	XmlRpc::XmlRpcValue::iterator iter = routes.begin();
	while( iter != routes.end() )
	{
		
		XmlRpc::XmlRpcValue::ValueStruct::value_type item = *iter;
		iter++;
		
		std::string routeName = item.first;
		
		std::string type = item.second["type"];
		std::string direction = item.second["direction"];
		std::string rosTopic = item.second["ros_topic"];
		std::string lcmMessage = item.second["lcm_channel"];

		ROS_INFO_STREAM( "Setting up route [" << routeName << "]"
			<< " between LCM [" << lcmMessage << "] and ROS [" 
			<< rosTopic << "]" << std::endl );
		

		Router::Ptr router;
		if( type.compare( "position" ) == 0 )
		{
			if( direction.compare( "ros_to_lcm" ) == 0 )
			{
				router = boost::make_shared<JointFeedbackMessageRouterR2L>( lcmMessage, lcm,
																			rosTopic, nh );
			}
			else if( direction.compare( "lcm_to_ros" ) == 0 )
			{
				router = boost::make_shared<JointCommandMessageRouterL2R>( lcmMessage, lcm,
																		   rosTopic, nh );
			}
			else
			{
				ROS_ERROR_STREAM( "Invalid direction. Must be ros_to_lcm or lcm_to_ros" );
				exit( -1 );
			}
				
		}
		else
		{
			ROS_ERROR_STREAM( "Invalid router type: " << type );
		}
		routers.push_back( router );
		
	}
		
	while( ros::ok() )
	{
		lcmSpinOnce(&lcm);
		ros::spinOnce();
	}
	exit( 0 );
	
}

void lcmSpinOnce(lcm::LCM *lcm)
{
	int rv;
	int fd = lcm->getFileno();
	fd_set readset;
	struct timeval tv = {0, 0};

	FD_ZERO(&readset);
	FD_SET(fd, &readset);
	rv = select(fd+1, &readset, NULL, NULL, &tv);
	while (rv > 0)
	{
		lcm->handle();
		rv = select(fd+1, &readset, NULL, NULL, &tv);
	}
	if (rv < 0)
	{
		ROS_ERROR_STREAM( "Bad return from lcmSpinOnce select call.");
	}
}
