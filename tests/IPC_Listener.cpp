#include "transfer_station/IPCInterface.h"
#include "transfer_station/PositionMessageI2R.h"

using namespace transfer_station;

void IPCCallback( void* data )
{
	
	IPCPositionMessage msg( "chatter" );
	msg.Deserialize( data );
	
	std::cout << "pose: " << msg.data.posedata.x << ", " << msg.data.posedata.y
		<< ", " << msg.data.posedata.theta << std::endl;
	
}

int main( int argc, char** argv )
{
	
	IPCInterface ipc;
	if( !ipc.Connect( "ipc_listener" ) )
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
	
	ipc.Subscribe( messageName, &IPCCallback );
	
	while( true )
	{
		ipc.Spin();
		usleep( 1E4 );
	}
	
}
