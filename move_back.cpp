#include <cmath>
#include <iostream>


static MathVector
move_back (int sensor)
{
  MathVector p;
  double b;
  double a;
  double c;
  double angle;

  c = 1;
  double rf = 3.14159 / 180;

  switch (sensor)
    {
    case 1:
      angle = 0 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 2:
      angle = 40 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 3:
      angle = 80 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 4:
      angle = 120 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 5:
      angle = 160 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 6:
      angle = 200 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 7:
      angle = 240 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 8:
      angle = 280 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    case 9:
      angle = 320 * rf;
      b = c * cos (angle);
      a = b * tan (angle);
      p.x = b;
      p.y = a;
      break;
    }

  p.x = -1 * p.x;
  p.y = -1 * p.y;
  return p;
}
