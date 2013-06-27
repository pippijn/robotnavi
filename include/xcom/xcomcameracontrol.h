//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _XCOMCAMERACONTROL_H_
#define _XCOMCAMERACONTROL_H_

#include <xcom/xcomcommand.h>
#include "robotinocom/robotinocom.h"

class XComCameraControl : public XComCommand
{
public:
  XComCameraControl();

  XComCameraControl( const XComCameraControl& other );
  XComCameraControl& operator=( const XComCameraControl& other );

  void setParameters( const RobotinoCameraParameters& param );
  RobotinoCameraParameters parameters() const;

private:
  bool customDecode( XComDataStreamDecoder& in );
  void customEncode( XComDataStreamEncoder& out );

  RobotinoCameraParameters _param;
};

#endif