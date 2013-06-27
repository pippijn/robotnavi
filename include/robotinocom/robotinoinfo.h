//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _ROBOTINOINFO_H_
#define _ROBOTINOINFO_H_

#include <string>

/**
* @brief	A class to deliver text messages from Robotino.
*
* @see		RobotinoCom::robotinoInfo
*/
class RobotinoInfo
{
public:
	//enum{ MaxMessageLength=2000 };
	static const int MaxMessageLength = 2000;

	RobotinoInfo()
		: _message( "not available" )
		, _isPassiveMode( false )
	{
	}

	void setMessage( const std::string& message )
	{
		_message = message;
	}

	std::string message() const
	{
		return _message;
	}

	void setPassiveMode( bool on )
	{
		_isPassiveMode = on;
	}

	bool isPassiveMode() const
	{
		return _isPassiveMode;
	}

private:
	std::string _message;
	bool _isPassiveMode;
};

#endif
