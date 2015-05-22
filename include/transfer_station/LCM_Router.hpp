#ifndef _XFERS_LCM_ROUTER_H_
#define _XFERS_LCM_ROUTER_H_

#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include <boost/shared_ptr.hpp>

#include <type_traits>

namespace transfer_station
{
	template<class L, class R>
	void lcm_to_ros( const L* l, R& r ) {
		static_assert( std::is_same<L,R>(), "You must specialize lcm_to_ros." );
	}

	template<class R, class L>
	void ros_to_lcm( const R& r, L& l ) {
		static_assert( std::is_same<L,R>(), "You must specialize ros_to_lcm." );
	}
	
	/*! \brief A LCM->ROS translation route. */
	template< class LCMMsg, class ROSMsg >
	class RouterL2R
	{
	public:
		
		typedef boost::shared_ptr<RouterL2R> Ptr;
		
		RouterL2R( const std::string& lcmName, lcm::LCM& lcm,
				const std::string& rosName, ros::NodeHandle& ros )
		: lcmChannelName( lcmName ), lcmInterface( lcm ),
		rosTopicName( rosName ), rosNode( ros )
		{
			rosPublisher = rosNode.advertise<ROSMsg>( rosTopicName, 1 );
			
			lcmInterface.subscribe( lcmChannelName, &RouterL2R::LCMCallback, this );
		}
		
		/*! \brief Callback used by LCM. */
		void LCMCallback( const lcm::ReceiveBuffer* rbuf,
						  const std::string& channel,
						  const LCMMsg* lcmMsg )
		{
			(void)channel;
			(void)rbuf;
			
			ROSMsg rosMsg;
			lcm_to_ros<LCMMsg, ROSMsg>( lcmMsg, rosMsg );
			
			rosPublisher.publish( rosMsg );
		}
		
	protected:
		
		/*! \brief The channel name on LCM. */
		std::string lcmChannelName;
		
		/*! \brief The associated LCM interface object. */
		lcm::LCM lcmInterface;
		
		/*! \brief The fully resolved ROS topic name. */
		std::string rosTopicName;
		
		/*! \brief The base level ROS node handle. */
		ros::NodeHandle rosNode;
		
		/*! \brief The ROS publisher for this router. */
		ros::Publisher rosPublisher;
		
	};
	
	/*! \brief A ROS->LCM translation route. */
	template< class ROSMsg, class LCMMsg >
	class RouterR2L
	{
	public:
		
		typedef boost::shared_ptr<RouterR2L> Ptr;
		
		RouterR2L( const std::string& lcmName, lcm::LCM& lcm,
				const std::string& rosName, ros::NodeHandle& ros )
		: lcmChannelName( lcmName ), lcmInterface( lcm ),
		rosTopicName( rosName ), rosNode( ros )
		{
			rosSubscriber = rosNode.subscribe<ROSMsg>( rosTopicName, 1, 
													   boost::bind( &RouterR2L<ROSMsg,LCMMsg>::ROSCallback, this, _1 ) );	
		}
		
		/*! \brief Callback used by ROS. */
		void ROSCallback( const typename ROSMsg::ConstPtr& rosMsg )
		{
			LCMMsg lcmMsg;
			ros_to_lcm<ROSMsg, LCMMsg>( *rosMsg, lcmMsg );
			
			lcmInterface.publish( lcmChannelName, &lcmMsg );
		}
		
	protected:
		
		/*! \brief The channel name on LCM. */
		std::string lcmChannelName;
		
		/*! \brief The associated LCM interface object. */
		lcm::LCM lcmInterface;
		
		/*! \brief The fully resolved ROS topic name. */
		std::string rosTopicName;
		
		/*! \brief The base level ROS node handle. */
		ros::NodeHandle rosNode;
		
		/*! \brief The ROS subscriber for this router. */
		ros::Subscriber rosSubscriber;
		
	};
	
}

#endif
