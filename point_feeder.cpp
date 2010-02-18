static CvPoint points[] = {
  { 1000,    0 },
  { 2780,  360 },
  { 4700,  360 },
  { 5900,   60 },
  { 6900,   60 },
};

struct point_feeder
{
  point_feeder ();
  CvPoint current ();
  void next ();

  bool first () const { return point == 0 || point == 1; }
  bool last  () const { return point == sizeof points / sizeof *points; }

  size_t point;
};

point_feeder::point_feeder ()
  : point (0)
{
}

CvPoint
point_feeder::current ()
{
  return points[point - 1];
}

void
point_feeder::next ()
{
  if (!last ())
    ++point;
}
