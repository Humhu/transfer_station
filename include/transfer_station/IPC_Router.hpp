#ifndef _XFERS_I2R_ROUTER_H_
#define _XFERS_I2R_ROUTER_H_

#include <ros/ros.h>
#include "transfer_station/IPC_Interface.h"

#include <boost/shared_ptr.hpp>

#include <type_traits>

namespace transfer_station
{
	
	template<class I, class R>
	void ipc_to_ros( const I& i, R& r ) {
		static_assert( std::is_same<I,R>(), "You must specialize ipc_to_ros." );
	}
	
	template<class R, class I>
	void ros_to_ipc( const R& r, I& i ) {
		static_assert( std::is_same<I,R>(), "You must specialize ros_to_ipc." );
	}
	
	/*! \brief A IPC->ROS translation route. */
	template< class IPCMsg, class ROSMsg >
	class RouterI2R
	{
	public:
		
		typedef boost::shared_ptr<RouterI2R> Ptr;
		
		RouterI2R( const std::string& ipcName, IPCInterface& ipc,
				const std::string& rosName, ros::NodeHandle& ros ) :
			ipcMessageName( ipcName ), ipcInterface( ipc ),
			rosTopicName( rosName ), rosNode( ros )
		{
			rosPublisher = rosNode.advertise<ROSMsg>( rosTopicName, 1 );
			
			define_message< IPCMsg >( ipcName, ipc );

			IPCInterface::MessageCallback cb =
					boost::bind( &RouterI2R<IPCMsg,ROSMsg>::IPCCallback, this, _1 );
			ipcInterface.Subscribe( ipcName, cb );
		}
		
		/*! \brief Callback used by IPC. */
		void IPCCallback( void* callData )
		{
// 			std::cout << "Router received IPC message [" << ipcMessageName << "]" << std::endl;
			IPCMsg ipcMsg;
			deserialize_message<IPCMsg>( callData, &ipcMsg );
			
			ROSMsg rosMsg;
			ipc_to_ros<IPCMsg, ROSMsg>( ipcMsg, rosMsg );
			
			rosPublisher.publish( rosMsg );
		}
		
	protected:
		
		/*! \brief The message name on IPC. */
		std::string ipcMessageName;
		
		/*! \brief The associated IPC interface object. */
		IPCInterface& ipcInterface;
		
		/*! \brief The fully resolved ROS topic name. */
		std::string rosTopicName;
		
		/*! \brief The base level ROS node handle. */
		ros::NodeHandle rosNode;
		
		/*! \brief The ROS publisher for this router. */
		ros::Publisher rosPublisher;
		
	};
	
	/*! \brief A ROS->IPC translation route. */
	template< class ROSMsg, class IPCMsg>
	class RouterR2I
	{
	public:
		
		typedef boost::shared_ptr<RouterR2I> Ptr;
		
		RouterR2I( const std::string& ipcName, IPCInterface& ipc,
				const std::string& rosName, ros::NodeHandle& ros )
		: ipcMessageName( ipcName ), ipcInterface( ipc ),
		rosTopicName( rosName ), rosNode( ros )
		{
			define_message< IPCMsg >( ipcName, ipc );
			
			rosSubscriber = rosNode.subscribe<ROSMsg>( rosTopicName, 1, 
						boost::bind( &RouterR2I<ROSMsg,IPCMsg>::ROSCallback, this, _1 ) );			
		}
		
		/*! \brief Callback used by ROS. */
		void ROSCallback( const typename ROSMsg::ConstPtr& rosMsg )
		{
// 			std::cout << "Router received ROS message [" << rosTopicName << "]" << std::endl;
			IPCMsg ipcMsg;
			ros_to_ipc<ROSMsg, IPCMsg>( *rosMsg, ipcMsg );
			
			ipcInterface.Publish( ipcMessageName, &ipcMsg );
		}
		
	protected:
		
		/*! \brief The message name on IPC. */
		std::string ipcMessageName;
		
		/*! \brief The associated IPC interface object. */
		IPCInterface& ipcInterface;
		
		/*! \brief The fully resolved ROS topic name. */
		std::string rosTopicName;
		
		/*! \brief The base level ROS node handle. */
		ros::NodeHandle rosNode;
		
		/*! \brief The ROS subscriber for this router. */
		ros::Subscriber rosSubscriber;
		
	};
	
}

#endif
