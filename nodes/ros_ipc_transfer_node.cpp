#include <ros/ros.h>

#include "transfer_station/IPCInterface.h"
#include "transfer_station/PositionMessageI2R.h"

using namespace transfer_station;

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "ros_ipc_transfer_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");
	
	// 1. Connect to IPC and set up
	std::string central, task;
	ph.param<std::string>( "ipc_central", central, "" );
	ph.getParam( "ipc_name", task );
	
	IPCInterface ipc;
	if( !ipc.Connect( task, central ) )
	{
		ROS_ERROR_STREAM( "Could not connect to central at " << central );
	}
	
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
		std::string ipcMessage = item.second["ipc_message"];

		ROS_INFO_STREAM( "Setting up route [" << routeName << "]"
			<< " between IPC [" << ipcMessage << "] and ROS [" 
			<< rosTopic << "]" << std::endl );
		
		
		Router::Ptr router;
		if( type.compare( "position" ) == 0 )
		{
			if( direction.compare( "ros_to_ipc" ) == 0 )
			{
				router = boost::make_shared<PositionMessageRouterR2I>( ipcMessage, ipc,
																	   rosTopic, nh );
			}
			else if( direction.compare( "ipc_to_ros" ) == 0 )
			{
				router = boost::make_shared<PositionMessageRouterI2R>( ipcMessage, ipc,
																	   rosTopic, nh );
			}
			else
			{
				ROS_ERROR_STREAM( "Invalid direction. Must be ros_to_ipc or ipc_to_ros" );
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
		ipc.Spin();
		ros::spinOnce();
	}
	exit( 0 );
	
}
