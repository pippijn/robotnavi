static int img_width;
static int img_height;

CvScalar const yellow = cvScalar (0, 255, 255);
CvScalar const black = cvScalar (0, 0, 0);
CvScalar const blue = cvScalar (255, 0, 0);
CvScalar const red = cvScalar (0, 0, 255);
CvScalar const orange = cvScalar (0, 165, 255);

// f(x) = (1 / (1 + e^(−5 ∙ (x − 1)))) ∙ (1 − 1 / (1 + e^(−5 ∙ (x − 10))))
#if 0
static double
calc_velocity (int target_diff, int origin_diff, bool first, bool last)
{
  int dist = abs (target_diff) + abs (origin_diff);

  double alpha = 0.005;
  double beta = 0.005;
  double x = abs (origin_diff);
  double x0 = 200;
  double x1 = dist - x0;

  double retval = 1;
  if (first)
    retval *=      (1. / (1. + pow (M_E, -alpha * (x - x0))));
  if (last)
    retval *= (1. - 1. / (1. + pow (M_E, -beta  * (x - x1))));

  //printf ("target_diff=%5d, origin_diff=%5d, a=%.6f\n", target_diff, origin_diff, retval);

  return retval;
}
#endif

struct robot
{
  static int const DEFAULT_ROT = 0;
  static int const DEFAULT_MOVEX = 0;
  static int const DEFAULT_MOVEY = 0;
  static int const navigator_step = 100;
  static int const max_speed = 300;
  static int const max_avoid_speed = 150;
  static int const warn_dist = 60;
  static int const panic_dist = 90;
  static int const dist_multiplier = 200;
  static int const override_step = 100;

  static int const room_h = 13;
  static int const room_w = 16;

  robot (std::string const& hostname);
  ~robot ();

  void run ();

  Navigator& nav ();

#if WITH_GUI
  void parse_image (RobotinoImage const* image);
  void print (int yadd, char const* fmt, ...);
  void draw_sensors ();
  void draw_statistics (int timediff, int distancex, int distancey);
#endif
  void update_position (int& timediff, int& distancex, int& distancey);
  void prevent_collision ();
  void avoid_obstacle (MathVector curtarget);
  bool panic ();
  void set_velocity ();

  void pop_navigator (int current, int count);
  void finished (int current, int count);

  void errorCb (int error);
  void connectedCb ();
  void connectionClosedCb ();

  std::vector<Navigator> navigators;
  IplImage* src;
  bool override;
  bool moving_back;
  bool local_finish;
  int rot;   // °/sec rotation
  double movex; // mm/sec (x)
  double movey; // mm/sec (y)
  double avoidx;
  double avoidy;

  MathVector position;
#if 0
  CvPoint origin;
  CvPoint target;
#endif

  timeval prev;
  RobotinoCom com;

  std::ofstream travel_log;
  std::ofstream command_log;
  std::ofstream speed_log;
};

robot::robot (std::string const& hostname)
  : src ()
  //, override (false)
  , override (true)
  , moving_back (false)
  , local_finish (false)
  , rot (DEFAULT_ROT)
  , movex (DEFAULT_MOVEX)
  , movey (DEFAULT_MOVEY)
  , avoidx (DEFAULT_MOVEX)
  , avoidy (DEFAULT_MOVEY)
  , position (0, 0, 0)
#if 0
  , origin ()
  , target ()
#endif
  , travel_log ("travel.log")
  , command_log ("command.log")
  , speed_log ("speed.log")
{
  if (!com.init ())
    throw 1;

  std::vector<gp_Pnt> pnts;
#if 0
  pnts.push_back (gp_Pnt (   0,    0, 0));
  pnts.push_back (gp_Pnt (2000,    0, 0));
  pnts.push_back (gp_Pnt (2000, 2000, 0));
  pnts.push_back (gp_Pnt (4000, 2000, 0));
#else
  pnts.push_back (gp_Pnt (   0,    0, 0));
  pnts.push_back (gp_Pnt (   0,  600, 0));
  pnts.push_back (gp_Pnt (   0, 1200, 0));
  pnts.push_back (gp_Pnt (   0, 1800, 0));
  pnts.push_back (gp_Pnt (   0, 2400, 0));
  pnts.push_back (gp_Pnt ( 600, 3600, 0));
  pnts.push_back (gp_Pnt (2400, 4200, 0));
  pnts.push_back (gp_Pnt (3600, 4200, 0));
  pnts.push_back (gp_Pnt (4800, 5400, 0));
  pnts.push_back (gp_Pnt (6000, 6600, 0));
  pnts.push_back (gp_Pnt (7200, 6000, 0));
  pnts.push_back (gp_Pnt (7800, 4800, 0));
  pnts.push_back (gp_Pnt (7800, 4200, 0));
  pnts.push_back (gp_Pnt (7200, 2400, 0));
  pnts.push_back (gp_Pnt (6600, 1200, 0));
  pnts.push_back (gp_Pnt (6000,  600, 0));
  pnts.push_back (gp_Pnt (4800,    0, 0));
#endif

  navigators.push_back (navigator_step);
  nav ().initialize (pnts);
  nav ().set_handler ((void(*)(void*, int, int))&robot::finished, this);

  com.setErrorCallback ((RobotinoErrorCb_t)&robot::errorCb, this);
  com.setConnectedCallback ((RobotinoConnectedCb_t)&robot::connectedCb, this);
  com.setConnectionClosedCallback ((RobotinoConnectionClosedCb_t)&robot::connectionClosedCb, this);

  std::cout << "Connecting to host " << hostname;
  com.connectToHost (hostname);

  while (com.state () == ConnectingState)
    {
      std::cout << "." << std::flush;
      usleep (20000);
    }
  std::cout << std::endl;

  if (com.error () != NoError)
    throw 1;

  std::cout << "Press q to quit." << std::endl;

  com.setImageRequest (true);

  RobotinoCameraParameters param = com.cameraParameters ();
  param.compression = HighCompression;
  param.resolution = VGA;
  param.brightness = 50;
  param.contrast = 50;
  com.setCameraParameters (param);

#if WITH_GUI
  cvNamedWindow ("Live Image", 1);
#endif
}

robot::~robot ()
{
#if WITH_GUI
  cvReleaseImage (&src);
  cvDestroyWindow ("Live Image");
#endif
}

void
robot::run ()
{
  gettimeofday (&prev, NULL);

#if WITH_GUI
  enum
  {
    NORMAL,
    CONTOUR,
  } conversion;
#endif

  int step = 0;
  while (true)
    {
#if WITH_GUI
      char c = cvWaitKey (200);
      switch (c)
        {
        case 'q':
          return;
        case 'a':
          rot += 10;
          break;
        case 'd':
          rot -= 10;
          break;
        case 'w':
          movex += override_step;
          break;
        case 's':
          movex -= override_step;
          break;
        case 'y':
          movey += override_step;
          break;
        case 'x':
          movey -= override_step;
          break;
        case 'z':
          movex = -movex;
          movey = -movey;
          break;
        case 'o':
          override = !override;
          break;
        case 'r':
          position.x = 0;
          position.y = 0;
          com.resetPosition (1, true);
          com.resetPosition (2, true);
          com.resetPosition (3, true);
          com.setOdometry (0, 0, 0);
        case 'e':
          rot = DEFAULT_ROT;
          movex = DEFAULT_MOVEX;
          movey = DEFAULT_MOVEY;
          break;
        case '1':
          conversion = NORMAL;
          break;
        case '2':
          conversion = CONTOUR;
          break;
        }
#else
      usleep (20 * 1000);
#endif

#if 0
      if (com.bumper ())
        {
          puts ("bump!");
          movex = -movex;
          movey = -movey;
          rot = -rot;
        }
#endif

      set_velocity ();
      int tdiff, distancex, distancey;
      update_position (tdiff, distancex, distancey);

      if (!com.update ())
        break;

#if WITH_GUI
      RobotinoImage const* image = com.lockImage ();
      if (image)
        {
          parse_image (image);

          IplImage* img = NULL;

          switch (conversion)
            {
            case NORMAL:
              draw_sensors ();
              draw_statistics (tdiff, distancex, distancey);

              img = src;
              break;

            case CONTOUR:
              IplImage* imgGrey = cvCreateImage (cvGetSize (src), IPL_DEPTH_8U, 1);
              cvCvtColor (src, imgGrey, CV_BGR2GRAY);
              IplImage* imgWhite = cvCreateImage (cvGetSize (src), IPL_DEPTH_8U, 1);
              cvThreshold (imgGrey, imgWhite, 240, 255, CV_THRESH_BINARY);
              IplImage* edgesImage = cvCreateImage (cvGetSize (src), IPL_DEPTH_8U, 1);
              cvCanny (imgWhite, edgesImage, 120, 120 * 3, 3);
              CvMemStorage* storage = cvCreateMemStorage (0);
              int const MAXIMUM_GAP = 10;
              CvSeq* overheadLines = cvHoughLines2 (edgesImage, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 360, 30, 10, MAXIMUM_GAP);

              img = edgesImage;
              break;
            }

          if (img)
            cvShowImage ("Live Image", img);

          com.unlockImage ();
        }
#endif

      travel_log << position.x << ',' << position.y << std::endl;
      speed_log << step << ',' << abs (movex) + abs (movey) << '\n';

      ++step;
    }
}

Navigator&
robot::nav ()
{
  assert (!navigators.empty ());
#if 0
  MathVector dest = navigators.back ().getDestination (gp_Pnt (position.x, position.y, 0));
  // we're there && we're not at the final point, yet (we have more navigators)
  if (dest.z == 666 && navigators.size () > 1)
    {
      navigators.pop_back ();
      return nav ();
    }
#endif
  return navigators.back ();
}

#if WITH_GUI
void
robot::parse_image (RobotinoImage const* image)
{
  Magick::Image img (Magick::Blob (image->data, image->dataSize));
#if 0
  Magick::Image route ("travel_log.png");
  {
#endif
    Magick::Geometry geo (800, 600);
    img.resize (geo);
#if 0
    route.resize (geo);
  }
  {
    route.matte (true);
    route.colorFuzz (7000);
    route.transparent (Magick::Color (USHRT_MAX, USHRT_MAX, USHRT_MAX, 0));
  }

  img.composite (route, 0, 0, Magick::BlendCompositeOp);
#endif

  unsigned int width = 0;
  unsigned int height = 0;

  if (image->parameters.type == JPG)
    {
      img_width = width = img.size ().width ();
      img_height = height = img.size ().height ();
    }
  else
    throw std::runtime_error ("only JPEG is supported");

  CvSize size = { width, height };

  if (src)
    {
      if (src->width != width || src->height != height)
        {
          cvReleaseImage (&src);
          src = cvCreateImage (size, IPL_DEPTH_8U, 3);
        }
    }
  else
    {
      src = cvCreateImage (size, IPL_DEPTH_8U, 3);
    }

  Magick::Blob data;
  img.write (&data, "bgr");
  memcpy (src->imageData, data.data (), data.length ());
}

void
robot::print (int yadd, char const* fmt, ...)
{
  CvFont font;
  double hScale = 0.3;
  double vScale = 0.3;
  int lineWidth = 1;
  cvInitFont (&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);

  char buf[100];
  va_list ap;
  va_start (ap, fmt);
  vsnprintf (buf, sizeof buf, fmt, ap);
  va_end (ap);

  if (yadd < 0)
    yadd = img_height + yadd - 20;

  if (yadd != 0)
    yadd += 2;
  cvRectangle (src,
               cvPoint (0, yadd),
               cvPoint (7 * strlen (buf), 18 + yadd),
               black, CV_FILLED);
  CvScalar colour = yellow;
  cvPutText (src, buf,
             cvPoint (10, 13 + yadd),
             &font, colour);
}

void
robot::draw_sensors ()
{
  double const X = img_width;
  double const Y = img_height;
  CvPoint const POINT1 = { (167. / 325.) * X, ( 18. / 325.) * Y };
  CvPoint const POINT2 = { ( 72. / 325.) * X, ( 52. / 325.) * Y };
  CvPoint const POINT3 = { ( 22. / 325.) * X, (139. / 325.) * Y };
  CvPoint const POINT4 = { ( 39. / 325.) * X, (238. / 325.) * Y };
  CvPoint const POINT5 = { (115. / 325.) * X, (302. / 325.) * Y };
  CvPoint const POINT6 = { (215. / 325.) * X, (302. / 325.) * Y };
  CvPoint const POINT7 = { (293. / 325.) * X, (239. / 325.) * Y };
  CvPoint const POINT8 = { (310. / 325.) * X, (140. / 325.) * Y };
  CvPoint const POINT9 = { (261. / 325.) * X, ( 53. / 325.) * Y };

  CvPoint curve1[] = { POINT1, POINT2, POINT3, POINT4, POINT5, POINT6, POINT7, POINT8, POINT9 };
    {
      CvPoint* curveArr[] = { curve1 };
      int nCurvePts[] = { 9 };
      int nCurves = 1;
      int isCurveClosed = 1;
      int lineWidth = 3;

      cvPolyLine (src, curveArr, nCurvePts, nCurves, isCurveClosed, override ? red : blue, lineWidth);
    }

    {
      CvFont font;
      double hScale = 0.3;
      double vScale = 0.3;
      int lineWidth = 1;
      cvInitFont (&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);

      size_t i = 0;
      assert (sizeof curve1 / sizeof *curve1 == 9);
      std::vector<std::string> sensors (com.numDistanceSensors ());

      foreach (CvPoint& point, curve1)
        {
          point.x -= 5;
          char buf[100];
          snprintf (buf, sizeof buf, "%3.0f", com.distance (i + 1) * dist_multiplier);
          sensors[i] = buf;
          cvRectangle (src,
                       cvPoint (point.x - 3, point.y - 10),
                       cvPoint (point.x + 20, point.y + 5),
                       black, CV_FILLED);
          CvScalar colour = yellow;
          if (com.distance (i + 1) * dist_multiplier > warn_dist)
            colour = orange;
          if (com.distance (i + 1) * dist_multiplier > panic_dist)
            colour = red;
          cvPutText (src, sensors[i].c_str (),
                     cvPoint (point.x, point.y),
                     &font, colour);
          ++i;
        }
    }
}

void
robot::draw_statistics (int timediff, int distancex, int distancey)
{
  if (src)
    {
      print (0,
             "t=%ld ms, "
             "v={%d,%d} mm/s, "
             "avoid_v={%d,%d} mm/s, "
             "d={%ld,%ld} mm, "
             "rot=%d deg/s"
             , timediff
             , int (movex)
             , int (movey)
             , int (avoidx)
             , int (avoidy)
             , distancex
             , distancey
             , rot
             );
      print (-1,
             "pos={%.0f,%.0f} cm "
#if 0
             "target={%d,%d} cm "
#endif
             , position.x / 10
             , position.y / 10
#if 0
             , target.x / 10
             , target.y / 10
#endif
             );
    }
}
#endif

void
robot::update_position (int& timediff, int& distancex, int& distancey)
{
  timeval now, diff;
  gettimeofday (&now, NULL);
  timersub (&now, &prev, &diff);
  gettimeofday (&prev, NULL);

  timediff = diff.tv_usec / 1000;
  distancex = ((movex + avoidx) * timediff) / 1000; // in mm
  distancey = ((movey + avoidy) * timediff) / 1000; // in mm

  position.x += distancex;
  position.y += distancey;

#if 0
  {
    std::ifstream s ("target.txt");
    int x = target.x;
    int y = target.y;
    if (s)
      s >> x >> y;
    if (x != target.x || y != target.y)
      {
        target.x = x;
        target.y = y;
        origin = position;
      }
  }
#endif

#define THRESHOLD 20
#if 0
  int target_diffx = position.x - target.x;
  int origin_diffx = position.x - origin.x;
  double velox = calc_velocity (target_diffx, origin_diffx, points.first (), points.last ());

  int target_diffy = position.y - target.y;
  int origin_diffy = position.y - origin.y;
  double veloy = calc_velocity (target_diffy, origin_diffy, points.first (), points.last ());

  double const max_v = 100;

  double base_vx = abs (target_diffx) + abs (origin_diffx);
  double base_vy = abs (target_diffy) + abs (origin_diffy);

  if (base_vx > DBL_EPSILON && base_vx < 50) base_vx = 50;
  if (base_vy > DBL_EPSILON && base_vy < 50) base_vy = 50;

  printf ("base_vx=%f, base_vy=%f\n", base_vx, base_vy);
  if (base_vx > max_v)
    {
      double correction = max_v / base_vx;
      printf ("base_vx=%f is too large for max=%f, multiplying with %f\n", base_vx, max_v, correction);
      base_vx *= correction;
      base_vy *= correction;
      printf (" => base_vx = %f, base_vy = %f\n", base_vx, base_vy);
    }
  if (base_vy > max_v)
    {
      double correction = max_v / base_vy;
      printf ("base_vy=%f is too large for max=%f, multiplying with %f\n", base_vy, max_v, correction);
      base_vx *= correction;
      base_vy *= correction;
      printf (" => base_vx = %f, base_vy = %f\n", base_vx, base_vy);
    }

  int ready = 0;
  if (abs (target_diffx) > THRESHOLD) // we're not there, yet
    movex = velox * (target_diffx < 0 ? base_vx : -base_vx);
  else
    {
      movex = DEFAULT_MOVEX;
      ++ready;
    }

  if (abs (target_diffy) > THRESHOLD) // we're not there, yet
    movey = veloy * (target_diffy < 0 ? base_vy : -base_vy);
  else
    {
      movey = DEFAULT_MOVEY;
      ++ready;
    }

  if (ready == 2)
    {
      origin.x = position.x;
      origin.y = position.y;
    }
#endif

  MathVector vec = nav ().getDestination (gp_Pnt (position.x, position.y, 0));
  while (local_finish)
    {
      navigators.pop_back ();
      assert (!navigators.empty ());

      local_finish = false;
      vec = nav ().getDestination (gp_Pnt (position.x, position.y, 0));
    }
  if (!override)
    {
      command_log << vec.x << ',' << vec.y << '\n';
#if 0
      printf ("pos={%f,%f} vec={%f,%f}\n"
              , position.x
              , position.y
              , vec.x
              , vec.y
             );
#endif
      movex = vec.x * max_speed / navigator_step;
      movey = vec.y * max_speed / navigator_step;

      if (movex >  max_speed) movex =  max_speed;
      if (movex < -max_speed) movex = -max_speed;
      if (movey >  max_speed) movey =  max_speed;
      if (movey < -max_speed) movey = -max_speed;

      //prevent_collision ();
      avoid_obstacle (vec);
    }

  set_velocity ();
}

void
robot::prevent_collision ()
{
  static std::vector<std::deque<float> > values (9);

  for (size_t i = 0; i < 9; i++)
    {
      values[i].push_back (com.distance (i + 1));
      if (values[i].size () > 3)
        values[i].pop_front ();
    }

  avoidx = 0;
  avoidy = 0;
  for (size_t i = 0; i < values.size (); i++)
    {
      std::deque<float> q = values[i];
      std::sort (q.begin (), q.end ());

      float avoid_v = com.distance (i + 1) * dist_multiplier;
      avoid_v *= 3;

      if (avoid_v > max_avoid_speed)
        avoid_v = max_avoid_speed;

      if (avoid_v > warn_dist)
        {
          printf ("warn_dist for sensor %d\n", i);
          MathVector vec = move_back (i + 1);
          avoidx += vec.x * avoid_v;
          avoidy += vec.y * avoid_v;
        }
    }

}

bool
robot::panic ()
{
  bool panic[9] = { 0 };
  if (   (panic[0] = com.distance (1) * dist_multiplier > panic_dist)
      || (panic[1] = com.distance (2) * dist_multiplier > panic_dist)
      || (panic[2] = com.distance (3) * dist_multiplier > panic_dist)
      || (panic[3] = com.distance (4) * dist_multiplier > panic_dist)
      || (panic[4] = com.distance (5) * dist_multiplier > panic_dist)
      || (panic[5] = com.distance (6) * dist_multiplier > panic_dist)
      || (panic[6] = com.distance (7) * dist_multiplier > panic_dist)
      || (panic[7] = com.distance (8) * dist_multiplier > panic_dist)
      || (panic[8] = com.distance (9) * dist_multiplier > panic_dist))
    {
      printf ("panic on sensors ");
      for (int i = 0; i < sizeof panic; i++)
        if (panic[i])
          printf ("%d:%f ", i + 1, com.distance (i + 1));
      printf (" at (%f,%f)\n", position.x, position.y);
      return true;
    }
  return false;
}

void
robot::avoid_obstacle (MathVector curtarget)
{
  if (moving_back)
    return;

  if (panic ())
    {
      Navigator newnav (navigator_step);

      MathVector unity = curtarget.unity ();
      unity.x *= -1000;
      unity.y *= -1000;
      MathVector const rev_target = position.add (unity);

      gp_Pnt const next_target = nav ().getNextTarget ();

      MathVector const rev_to_next ( (next_target.X () - rev_target.x) / 2
                                   , (next_target.Y () - rev_target.y) / 2
                                   , 0
                                   );
      MathVector const ovec = rev_to_next.getOrthogonalVector ();
      MathVector const med_target = rev_target.add (rev_to_next).add (ovec);

      std::vector<gp_Pnt> points;
      points.push_back (gp_Pnt (position.x, position.y, 0));
      points.push_back (rev_target.getGP_Pnt ());
      points.push_back (med_target.getGP_Pnt ());
      points.push_back (next_target);

      printf ("new route:\n"
              "1: (%f,%f)\n"
              "2: (%f,%f)\n"
              "3: (%f,%f)\n"
              "4: (%f,%f)\n"
              , position.x, position.y
              , rev_target.x, rev_target.y
              , med_target.x, med_target.y
              , next_target.X (), next_target.Y ()
              );
#if 0
      travel_log << position.x << ',' << position.y << '\n';
      travel_log << rev_target.x << ',' << rev_target.y << '\n';
      travel_log << med_target.x << ',' << med_target.y << '\n';
      travel_log << next_target.X () << ',' << next_target.Y () << '\n';
      throw 0;
#endif

      navigators.push_back (navigator_step);
      nav ().initialize (points);
      nav ().set_handler ((void(*)(void*, int, int))&robot::pop_navigator, this);

      moving_back = true;
    }
}

void
robot::set_velocity ()
{
  // max hardware velocity
  if (movex >  1400) movex =  1400;
  if (movex < -1400) movex = -1400;
  if (movey >  1400) movex =  1400;
  if (movey < -1400) movex = -1400;

  if (!override)
    {
      if (movex >  max_speed) movex =  max_speed;
      if (movex < -max_speed) movex = -max_speed;
      if (movey >  max_speed) movey =  max_speed;
      if (movey < -max_speed) movey = -max_speed;
    }
  printf ("set_velocity (%f, %f, %d)\n", movex, movey, rot);
  com.setVelocity (movex + avoidx,      // vx = mm/s
                   movey + avoidy,      // vy = mm/s
                   rot);                // omega = deg/s
}

void
robot::pop_navigator (int current, int count)
{
  printf ("pop_navigator (%d, %d)\n", current, count);
  if (current == count)
    local_finish = true;
  else if (current == 1)
    moving_back = false;
}

void
robot::finished (int current, int count)
{
  printf ("finished (%d, %d)\n", current, count);
  if (current == count)
    {
      movex = DEFAULT_MOVEX;
      movey = DEFAULT_MOVEY;
      override = true;
    }
}

void
robot::errorCb (int error)
{
  std::cout << "\nError " << RobotinoCom::errorString (error) << '\n';
  exit (1);
}

void
robot::connectedCb ()
{
  std::cout << "\nConnected\n";
}

void
robot::connectionClosedCb ()
{
  std::cout << "\nConnection closed\n";
}
