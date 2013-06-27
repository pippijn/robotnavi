//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _REC_ROBOTINO_COM_COM_H_
#define _REC_ROBOTINO_COM_COM_H_

#include "rec/robotino/com/ComId.h"
#include "rec/robotino/com/RobotinoException.h"
#include <string>

namespace rec
{
	namespace robotino
	{
		namespace com
		{
			class ComImpl;

			/**
			* @brief	Represents a communication device.
			*/
			class
#ifdef WIN32
#  ifdef rec_robotino_com_EXPORTS
		__declspec(dllexport)
#endif
#  ifdef rec_robotino_com2_EXPORTS
		__declspec(dllexport)
#endif
#endif
			Com
			{
			public:
				/** \brief Communication errors.

				\li \c NoError											no error
				\li \c ErrorConnectionRefused				if the connection was refused
				\li \c ErrorSocketSend							if a send to the socket failed
				\li \c ErrorSocketRead							if a read from the socket failed
				\li \c ErrorImageServerCreate				if the creation of the image server failed
				\li \c ErrorImageServerBind					if the image server could not bind to the imagePort()
				\li \c ErrorTimeout									if Robotino did not send something for more than timeout() milliseconds
				\li \c ErrorConnectionBusy					if Robotino is already connected to someone else
				\li \c ErrorClientCreate						if the creation of the client socket fails
				\li \c ErrorHostNotFound						if a DNS lookup fails
				\li \c ErrorUndefined								undefined error
				\li \c ErrorWinsock2Initialization	only on Win32 systems if winsock2 could not be initialised
				*/
				typedef enum
				{
					NoError,
					ErrorConnectionRefused,
					ErrorSocketSend,
					ErrorSocketRead,
					ErrorImageServerCreate,
					ErrorImageServerBind,
					ErrorTimeout,
					ErrorConnectionBusy,
					ErrorClientCreate,
					ErrorHostNotFound,
					ErrorUndefined,
					ErrorWinsock2Initialization
				} Error;

				/** \brief State of connection.

				\li \c NotConnected										No connection to Robotino established
				\li \c Connecting											Connection setup in progress
				\li \c Connected											Connection successfully established
				*/
				typedef enum
				{
					NotConnected,
					Connecting,
					Connected
				} ConnectionState;

				Com();
				
				virtual ~Com();

				/**
				* The unique identifier of this communication object.
				*
				* @return	The identifier.
				* @throws	nothing.
				*/
				ComId id() const;

				/**
				* Connects to the server. This function blocks until the connection is established or an error occurs.
				* @param isBlocking If true, this function blocks until the connection is established or an error occurs.
				*										Otherwise the function is non blocking and you have to watch for error or connected callbacks.
				* @throws		ComException If the client couldn't connect.
				*/
				void connect( bool isBlocking = true );

				/**
				* Disconnects from the server and disables autoupdating.
				*
				* @throws	nothing.
				*/
				void disconnect();

				/**
				* Test wether the client is connected to the server.
				* @return	TRUE if connected, FALSE otherwise.
				* @throws	nothing.
				*/
				bool isConnected() const;

				/**
				* Sets the address of the server.
				* @param address	The address of the server e.g. "127.0.0.1"
				* @see				address
				* @throws			nothing.
				*/
				void setAddress( std::string address = "172.26.1.1" );

				/**
				* Returns the currently active server address.
				* 
				* @return	Address set with setAddress
				* @see		setAddress
				* @throws	nothing.
				*/
				std::string address() const;

				/**
				* Sets the destination port of the image streamer. Does nothing if
				* we're already connected to a server.
				* @param port	The destination port of the image streamer e.g. 8080
				* @throws		nothing.
				*/
				void setImageServerPort( unsigned int port = 8080 );

				/**
				* @return The current state of connection
				*/
				ConnectionState connectionState() const;

				/**
				* This function is called on errors.
				* Note: This function is called from outside the applications main thread.
				* This is extremely important particularly with regard to GUI applications.
				*@param error The error which caused this event.
				* @throws		nothing.
				*/
				virtual void errorEvent( Error error );

				/**
				* This function is called if a connection to Robotino has been established.
				* Note: This function is called from outside the applications main thread.
				* This is extremely important particularly with regard to GUI applications.
				* @throws		nothing.
				*/
				virtual void connectedEvent();

				/**
				* This function is called if a connection is closed.
				* Note: This function is called from outside the applications main thread.
				* This is extremely important particularly with regard to GUI applications.
				* @throws		nothing.
				*/
				virtual void connectionClosedEvent();

				/**
				* This function is called whenever the connection state changes
				* @param newState The new connection state
				* @param oldState The previous connection state
				*/
				virtual void connectionStateChangedEvent( ConnectionState newState, ConnectionState oldState );

			private:
				ComImpl* _impl;
			};

			/**
			* @brief	RobotinoException exclusive to Com objects.
			*/
			class ComException : public RobotinoException
			{
			public:
				ComException( Com::Error e, const std::string& message )
					: RobotinoException( message )
					, error( e )
				{
				}

				const Com::Error error;
			};
		}
	}
}
#endif
