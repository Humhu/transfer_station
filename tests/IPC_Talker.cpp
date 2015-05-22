#include "transfer_station/IPCInterface.h"
#include "transfer_station/PositionMessageI2R.h"

using namespace transfer_station;
	
	
int main( int argc, char** argv )
{
	
	IPCInterface ipc;
	if( !ipc.Connect( "ipc_talker" ) )
	{
		std::cerr << "Could not connect to central." << std::endl;
		return -1;
	}
	
	std::string messageName;
	if( argc < 2 )
	{
		messageName = "chatter";
	}
	else
	{
		messageName = std::string( argv[1] );
	}
	
	IPCPositionMessage::Define( messageName, ipc );
	
	unsigned int seq = 0;
	while( true )
	{
		
		IPCPositionMessage msg( messageName );
		msg.data.posedata.x = seq*0.1;
		msg.data.posedata.y = seq*0.2;
		msg.data.posedata.theta = seq*-0.1;
		ipc.Publish( msg );
		seq++;
		
		std::cout << "pose: " << msg.data.posedata.x << ", " << msg.data.posedata.y
			<< ", " << msg.data.posedata.theta << std::endl;
		
		ipc.Spin();
		usleep( 1E5 );
	}
	
}
