#include <csignal>
#include <cstdlib>
#include <cstdarg>
#include <pthread.h>
#include <sys/time.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/foreach.hpp>

#include <festival.h>

#if WITH_GUI
#include <Magick++.h>
#endif
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <robotinocom/robotinocom.h>

#define foreach BOOST_FOREACH

#include "navigation/Navigator.h"

#include "move_back.cpp"
#include "robot.cpp"

int
main (int argc, char** argv)
{
  std::string hostname = "172.26.1.2";

  if (argc > 1)
    hostname = argv[1];

  try
  {
    robot robot (hostname);
    robot.run ();
  }
  catch (int)
  {
  }

  return 0;
}
