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

double CCurve::Evaluate(double x, double y)
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

bool CCurve::getNext(int looped)
{
  // is true, when it's the first call of the function, otherwise false
  static bool firstCall = true;
  // the loop count (how many times the curve looped already?)
  static int loopCount = 0;
  // the return value
  bool returnValue = true;

  if (c_err || (cur == -1))
    return false;

  
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
  else if (looped)
  {
    // cur is second last and looped is activated
    cur = 0;
    loopCount++;
    if (loopCount == looped)
    {
      returnValue = false;
    };
  }
  else
  {
    // it's the second last point -> do nothing
    returnValue = false;
  }

  if (returnValue)
  {
    recompute_coeffs(cur);
  }
  else
  {
    // reset everything
    firstCall = true;
    cur = -1;
    loopCount = 0;
  }

  ROS_INFO("Next point: %d [%f, %f]", cur, tx[cur], ty[cur]);

  return returnValue;
}

int CCurve::getPrev(int looped)
{
  static int ft = 1;
  int ret_val = 1;

  if (c_err || (cur == -1))
    return 0;

  switch (cur)
  {
    case 0:
      (ft == 1) ? (ft = 0) : (cur++);
      break;
    default:
      if (cur > 0)
      {
        cur--;
      }
      else
      {
        (looped) ? (cur = count - 2) : (ret_val = 0);
      }
  };

  if (ret_val)
  {
    recompute_coeffs(cur);
  }
  else
  {
    ft = 1;
    cur = -1;
  }

  ROS_INFO("Prev Point: %d [%f, %f]", cur, tx[cur], ty[cur]);

  return ret_val;
}

int CCurve::addNextPoint(double x, double y)
{
  if (!c_err)
  {
    if (add_c < count)
    {
      tx[add_c] = x;
      ty[add_c] = y;
      add_c++;
      return 1;
    }
    else
      return 0;
  }
  else
    return 0;
}

int CCurve::getSegmentNumber()
{
  if (c_err)
  {
    return -1;
  }

  return cur;
}

int CCurve::pointInn(double x, double y)
{
  double a1, b1, c1, c2, c3;
  if (c_err)
  {
    return 0;
  }

  a1 = tx[cur + 1] - tx[cur];
  b1 = ty[cur + 1] - ty[cur];
  c1 = -a1 * tx[cur] - b1 * ty[cur];
  c2 = -a1 * tx[cur + 1] - b1 * ty[cur + 1];
  c3 = -a1 * x - b1 * y;

  if (c2 >= c1)
  {
    if (c3 >= c1)
      return 1;
    else
      return 0;
  }
  else
  {
    if (c2 <= c1)
    {
      if (c3 >= c2)
        return 1;
      else
        return 0;
    }
    else
      return 0;
  }
}

double CCurve::getDistance(double x, double y)
{
  double res;
  if (c_err)
  {
    return nan("NAN");
  }

  res = fabs(A * x + B * y + C) / sqrt(A * A + B * B);
  return res;
}

double CCurve::getDistanceToEnd(double x, double y)
{
  double t1, t2, res;

  if (c_err)
  {
    return nan("NAN");
  }

  t1 = this->getDistance(x, y);
  t2 = (tx[cur + 1] - x) * (tx[cur + 1] - x) + (ty[cur + 1] - y) * (ty[cur + 1] - y);
  res = sqrt(t2 - t1 * t1);
  return res;
}
