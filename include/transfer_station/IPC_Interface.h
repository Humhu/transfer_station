#ifndef _XFERS_INTERFACE_H_
#define _XFERS_INTERFACE_H_

#include <ipc.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <unordered_map>

#include "transfer_station/Serializer.h"

namespace transfer_station
{

	/*! \brief Wrapper class around IPC. */
	class IPCInterface
	{
	public:
		
		typedef boost::shared_ptr<IPCInterface> Ptr;
		typedef boost::function<void(void*)> MessageCallback;
		
		IPCInterface()
			: connected( false ) 
		{}
		
		~IPCInterface()
		{
			Disconnect();
		}
		
		/*! \brief Connect this interface with specified name to central. */
		bool Connect( const std::string& task, const std::string& central = "" )
		{
			Disconnect();
			
			IPC_RETURN_TYPE retval;
			if( central.empty() )
			{
				retval = IPC_connectModule( task.c_str(), NULL );
			}
			else
			{
				retval = IPC_connectModule( task.c_str(), central.c_str() );
			}
			
			if( retval == IPC_Error )
			{
				return false;
			}
			taskName = task;
			centralName = central;
			connected = true;
			return true;
		}
		
		/*! \brief Disconnect from central. */
		void Disconnect()
		{
			if( !connected ) { return; }
			IPC_disconnect();
		}
		
		/*! \brief Check IPC callback queue forever. [may be only once] */
		void Spin()
		{
			IPC_listen( 0 );
		}
		
		/*! \brief Define a message topic given the format string. */
		void DefineMessage( const std::string& mname, const std::string& fstring )
		{
			IPC_RETURN_TYPE ret = 
			IPC_defineMsg( mname.c_str(), IPC_VARIABLE_LENGTH, fstring.c_str() );
			
			if( ret != IPC_OK )
			{
				throw std::runtime_error( "Definition of message [" + mname + 
				"] with format string: " + fstring + " failed!" );
			}
			
			if( IPC_checkMsgFormats( mname.c_str(), fstring.c_str() ) != IPC_OK )
			{
				throw std::runtime_error( "Verification of message [" + mname + 
				"] with format string: " + fstring + " failed!" );
			}
		}
		
		/*! \brief Define a format given the format string. */
		void DefineFormat( const std::string& fname, const std::string& fstring )
		{
			IPC_RETURN_TYPE ret =
			IPC_defineFormat( fname.c_str(), fstring.c_str() );
			if( ret != IPC_OK )
			{
				throw std::runtime_error( "Definition of format [" + fname + 
				"] with format string: " + fstring + " failed!" );
			}
		}
		
		/*! \brief Register a callback to a topic name. */
		void Subscribe( const std::string& mname, MessageCallback cb )
		{
			IPC_subscribeData( mname.c_str(), &IPCInterface::MessageHandler, this );
			callbacks[ mname ].push_back( cb );
		}
		
		/*! \brief Attempt to transmit a message. */
		template< class T>
		bool Publish( const std::string& name, const T* msg )
		{
			char buff[ sizeof(T) ];
			Serializer ser( buff );
			ser.Write<T>( msg );
			IPC_RETURN_TYPE ret = IPC_publishData( name.c_str(), buff );
			return ret == IPC_OK;
		}
		
	private:
		
		bool connected;
		std::string taskName;
		std::string centralName;
		
		typedef std::unordered_map< std::string, std::vector<MessageCallback> > CallbackRegistry;
		CallbackRegistry callbacks;
		
		/*! \brief Callback used by IPC. */
		static void MessageHandler( MSG_INSTANCE msgInstance, void* callData,
									void* clientData )
		{
			IPCInterface* obj = static_cast<IPCInterface*>( clientData );
			std::string msgName( IPC_msgInstanceName( msgInstance ) );
			
			CallbackRegistry::iterator iter;
			iter = obj->callbacks.find( msgName );
			if( iter == obj->callbacks.end() )
			{
				throw std::runtime_error( "Received message " + msgName + 
				" with no registered callback." );
			}
			
			std::vector<MessageCallback> cbs = iter->second;
			BOOST_FOREACH( MessageCallback& cb, cbs )
			{
				cb( callData );
			}
			
			FORMATTER_PTR formatter;
			formatter = IPC_msgInstanceFormatter( msgInstance );
			IPC_freeData( formatter, callData );
		}
		
	};

	/*! \brief Returns the format name for an IPC format. Must be specialized. */
	template< class T >
	std::string get_format_name() 
	{
		static_assert( !std::is_same<T,T>(), "You must specialize get_format_name()" );
		return "";
	}

	/*! \brief Returns the format specifier string for an IPC format. Must be specialized. */
	template< class T >
	std::string get_format_specifier() 
	{
		static_assert( !std::is_same<T,T>(), "You must specialize get_format_specifier()" );
		return "";
	}
	
	/*! \brief Declares an IPC format with the specified interface. */
	template< class T >
	void define_format( IPCInterface& ipc )
	{
		ipc.DefineFormat( get_format_name<T>(), get_format_specifier<T>() );
	}
	
	/*! \brief Returns the specifier string for an IPC message type. Must be specialized. */
	template< class T >
	std::string get_message_specifier() 
	{
		return get_format_specifier<T>();
	}
	
	/*! \brief Declares all the IPC formats used by an IPC message type on the specified interface. 
	 * Must be specialized. */
	template< class T >
	void define_message_formats( IPCInterface& ipc ) {
		static_assert( !std::is_same<T,T>(), "You must specialize define_message_formats()" );
	}
	
	/*! \brief Reads a byte array into a message struct. */
	template< class T >
	void deserialize_message( void* src, T* dst ) {
		Deserializer des( src );
		des.Read<T>( dst, 1 );
	}
	
	/*! \brief Writes a message struct into a byte array. */
	template< class T >
	void serialize_message( const T* src, void* dst ) {
		Serializer ser( dst );
		ser.Write<T>( src );
	}
	
	/*! \brief Defines a message topic on the specified interface. */
	template< class T>
	void define_message( const std::string& name, IPCInterface& ipc )
	{
		define_message_formats<T>( ipc );
		ipc.DefineMessage( name, get_message_specifier<T>() );
	}
	
}

#endif
