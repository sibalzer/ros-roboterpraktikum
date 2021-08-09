#include "curves.h"

using namespace std;

CCurve::CCurve(int num) : count(num), add_c(0)
{
  if (count <= 0)
  {
    count = 0;
    c_err = 1;
    cur = -1;
  }
  else
  {
    tx = new double[count];
    ty = new double[count];
    c_err = 0;
    cur = -1;
  }
}

CCurve::~CCurve()
{
  if (!c_err)
  {
    delete[] tx;
    delete[] ty;
  }
}

bool CCurve::WriteToFile(const char* fname)
{
  ofstream pth;
  int i;
  if (c_err)
  {
    return false;
  }

  pth.open(fname);
  if (!pth)
    return false;

  // print out number of points
  pth << count << endl;

  // print out coordinates of points
  for (i = 0; i < count; i++)
    pth << tx[i] << " " << ty[i] << endl;

  pth.close();
  return true;
}

bool CCurve::LoadFromFile(const char* fname)
{
  ifstream file1;
  char buff[255];
  int i = 0;

  file1.open(fname);
  if (!file1)
  {
    file1.close();
    return false;
  }

  // load count from file (number of points)
  file1 >> count;
  file1.getline(buff, 255);
  if (file1.eof())
  {
    file1.close();
    return false;
  }

  if (!c_err)
  {
    delete[] tx;
    delete[] ty;
    count = 0;
    c_err = 1;
    cur = -1;
  };

  tx = new double[count];
  ty = new double[count];

  // read all points from file
  do
  {
    tx[i] = ty[i] = nan("NAN");
    file1 >> tx[i];
    file1 >> ty[i];
    file1.getline(buff, 255);

    if (!isnan(tx[i]) && !isnan(ty[i]))
    {
      // optionally scale points
      // tx[i] *= 0.01;
      // ty[i] *= 0.01;
      i++;
    }
    else
    {
      break;
    }
  } while (!file1.eof() && i < count);

  if (i != count)
  {
    return false;
  }

  // everything has successfully finished
  c_err = 0;
  return (count != 0);
};

double CCurve::evaluate(const double x, const double y) const
{
  if (c_err)
    return nan("NAN");
  return A * x + B * y + C;
}

double CCurve::getAng()
{
  if (c_err || (cur == -1))
  {
    return nan("NAN");
  }

  return atan2(ty[cur + 1] - ty[cur], tx[cur + 1] - tx[cur]);
}

double CCurve::getTan()
{
  if (c_err || (cur == -1))
  {
    return nan("NAN");
  }

  return atan2(ty[cur + 1] - ty[cur], tx[cur + 1] - tx[cur]);
}

bool CCurve::initTraversal()
{
  if (c_err)
  {
    return false;
  }

  cur = 0;

  ROS_INFO("Traversal initialized");
  return true;
}

bool CCurve::getNext(const int looped)
{
  // is true, when it's the first call of the function, otherwise false
  static bool firstCall = true;
  // the loop count (how many times the curve looped already?)
  static int loopCount = 0;
  // the return value
  bool hasCurUpdated = true;

  if (c_err || (cur == -1))
  {
    return false;
  }

  if (!firstCall && cur < count - 2)
  {
    // it's not the first call and cur is not the second last
    cur++;
  }
  else if (firstCall)
  {
    // it's the first call -> do nothing
    firstCall = false;
  }
  else if (looped != 0)
  {
    if (loopCount >= looped)
    {
      hasCurUpdated = false;
    } else {
      cur = 0;
      loopCount++;
    }
  }
  else
  {
    // it's the second last point -> do nothing
    hasCurUpdated = false;
  }

  if (hasCurUpdated)
  {
    updateCoefficients();
  }
  else
  {
    // reset everything
    firstCall = true;
    cur = -1;
    loopCount = 0;
  }

  ROS_INFO("Path: %d -> %d [%f, %f] -> [%f, %f]",
    cur, cur+1, tx[cur], ty[cur], tx[cur+1], ty[cur+1]);

  return hasCurUpdated;
}

bool CCurve::addNextPoint(const double x, const double y)
{
  if (c_err || add_c >= count)
  {
    return false;
  }

  tx[add_c] = x;
  ty[add_c] = y;
  add_c++;
  return true;
}

bool CCurve::pointIn(const double x, const double y) const
{
  if (c_err)
  {
    return 0;
  }

  const double diffX = tx[cur + 1] - tx[cur];
  const double diffY = ty[cur + 1] - ty[cur];

  // boundary of starting point
  const double c1 = -diffX * tx[cur    ] - diffY * ty[cur    ];
  // boundary of destination point
  const double c2 = -diffX * tx[cur + 1] - diffY * ty[cur + 1];
  // level of current point
  const double c3 = -diffX * x           - diffY * y;

  return c3 >= min(c1, c2);
}

double CCurve::getDistance(double x, double y) const
{
  if (c_err)
  {
    return nan("NAN");
  }

  return fabs(evaluate(x, y)) / sqrt(A * A + B * B);
}

double CCurve::getDistanceToEnd(const double x, const double y) const
{
  if (c_err)
  {
    return nan("NAN");
  }

  const double diffX = tx[cur + 1] - x;
  const double diffY = ty[cur + 1] - y;
  const double t1 = getDistance(x, y);
  const double t2 = diffX * diffX + diffY * diffY;

  return sqrt(t2 - t1 * t1);
}
