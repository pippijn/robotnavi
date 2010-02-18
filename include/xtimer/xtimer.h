//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _X_TIMER_H_
#define _X_TIMER_H_

#ifdef WIN32
#ifdef xtimer_EXPORTS
  #define XTIMER_EXPORT __declspec(dllexport)
#else
  #define XTIMER_EXPORT //__declspec(dllimport)
#endif
#else
#define XTIMER_EXPORT
#endif

class XTimerImpl;

class XTIMER_EXPORT XTimer
{
public:
  XTimer();
  ~XTimer();
  
  /**Start the timer. If the timer is already running, i.e. isNull() will return false, the timer is restarted.*/
  void start();

  /**Milliseconds since the last call to start(). Returns 0 if start() has not been called before.*/
  float msecsElapsed() const;
  
  /**1000 / msecsElapsed(). Returns 0 if start() has not been called before.*/
  unsigned int frequency();

  /**Resets the timer. isNull() returns true afterwards.*/
  void reset();

  /**Returns false if timing is started by a call to start(). Returns true before a call to start() or after
  calling reset().*/
  bool isNull() const;

  /* returns a global time. unit is ms */
  static double globalTimestamp();

private:
  XTimerImpl* _impl;
  static XTimer _globalTimer;
};

#endif

