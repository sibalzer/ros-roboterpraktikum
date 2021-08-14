#include "gio_path.h"

using namespace std;

void CGioController::InitDefault()
{
  this->isRunning = 0;
  this->AXIS_LENGTH = 0.485;
  this->Vm = 1;
  this->d_y = 0.001;
  this->d_th = 0.34;
  this->kr_max = 2 / AXIS_LENGTH;
  this->u0 = Vm / 4.0;
  this->a = 1.2;
  this->epsilon = 1e-4;
  this->endDistanceMax = 0.1;
  this->path = new CCurve();
  this->setPose(0.0, 0.0, 0);
  this->setLocalSystem(0.0);
}

void CGioController::setLocalSystem(double ang)
{
  ex[0] = cos(ang);
  ex[1] = sin(ang);
  ey[0] = -sin(ang);
  ey[1] = cos(ang);
}

double CGioController::H_case_1(double y, double theta, double u, double alpha, double* gamma)
{
  double h_j = SQR(kr_max) / (SQR(theta) * SQR(1 + 2 * alpha));
  *gamma = 2 * alpha * u * sqrt(h_j);
  return h_j;
};

double CGioController::H_case_2(double y, double theta, double u, double alpha, double* gamma)
{
  double h_j =
      0.5 * (-SQR(theta) / SQR(y) + sqrt(SQR(SQR(theta) / SQR(y)) + 4 * SQR(kr_max) / (SQR(y) * SQR(1 + 2 * alpha))));
  *gamma = 2 * alpha * u * sqrt(h_j);
  return h_j;
};

double CGioController::Compute_W(double y, double theta, double a, double u, int* err)
{
  double res, h, gamma;

  if (fabs(y) < epsilon)
  {  // e. g. y = 0
    if (fabs(theta) >= d_th)
    {
      // cerr << "H1_1\n";
      h = H_case_1(y, theta, u, a, &gamma);
      *err = 0;
    }
    else
    {  // value for theta to small to evaluate correctly
      // cerr << "H1_2\n";
      h = H_case_1(y, d_th, u, a, &gamma);
      *err = 0;
    }
  }
  else
  {
    if (fabs(y) >= d_y)
    {                                       // value of y is big enough
      h = H_case_2(y, d_th, u, a, &gamma);  // why d_th?
      *err = 0;
    }
    else
    {  // value of y too small
      if (fabs(theta) < d_th)
      {  // value of theta too small
        h = H_case_2(d_y, d_th, u, a, &gamma);
        *err = 0;
      }
      else
      {  // value of theta ok
        h = H_case_2(d_y, theta, u, a, &gamma);
        *err = 0;
      }
    }
  }
  // use the simplest equation possible to determine omega
  if (fabs(theta) >= d_th)
  {
    res = -h * u * y * sin(theta) / theta - gamma * theta;
  }
  else
  {
    if (fabs(theta) <= d_th && fabs(theta) >= d_th_0)
    {
      res = -h * u * y - gamma * theta;
    }
    else
    {
      res = -h * u * y;
    }
  }

  return res;
}

CGioController::CGioController()
{
  this->InitDefault();
  giofile.open("pos.dat");
};

CGioController::~CGioController()
{
  giofile.flush();
  giofile.close();
  giofile.clear();
  delete path;
}

void CGioController::setAxisLength(double val)
{
  if (fabs(val) > 0)
  {
    this->AXIS_LENGTH = fabs(val);
    this->kr_max = 2 / AXIS_LENGTH;
  }
}

double CGioController::getAxisLength()
{
  return this->AXIS_LENGTH;
}

void CGioController::setCurrentVelocity(double val, int abs)
{
  if (abs)
  {
    this->u0 = val;
  }
  else
  {
    this->u0 = (this->u0 > val) ? (val + this->u0) : (this->u0 + val);
  }
  this->Vm = this->u0 * 2.0;
}

double CGioController::getCurrentVelocity()
{
  return this->u0;
}

void CGioController::setPose(double x, double y, double phi)
{
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NORMALIZE_ANGLE(this->phi0);
}

void CGioController::getPose(double& x, double& y, double& ph)
{
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}

int CGioController::getPathFromFile(const char* fname)
{
  int res = path->LoadFromFile(fname);
  if (res)
  {
    path->initTraversal();
    this->isRunning = path->getNext();
  }
  return res;
}

bool CGioController::canDetermineRobotPosition(int looped)
{
  while (isRunning)
  {
    const bool isInBoundary = path->pointIn(x0, y0);
    const bool isNotOnDestination = path->getDistanceToEnd(x0, y0) > endDistanceMax;
    if (isInBoundary && isNotOnDestination) {
      ROS_DEBUG("In boundary and not on destination. Continue...");
      giofile << path->getDistanceToEnd(x0, y0) << " ";  // u 1
      return true;
    }
    
    ROS_DEBUG("Get next point");
    isRunning = path->getNext(looped);
  }

  return false;
}

bool CGioController::getNextState(double& u, double& w, double& vleft, double& vright, const int looped)
{
  double l, phic, pathAng, tmpw;
  int err;

  if (!canDetermineRobotPosition(looped))
  {
    u = 0;
    w = 0;
    vleft = 0;
    vright = 0;
    return false;
  }

  l = path->getDistance(x0, y0);
  /// WHY?????
  if (path->evaluate(x0, y0) > 5e-7)
  {
    l = -l;
  }

  giofile << l << " " << path->evaluate(x0, y0) << " ";  // u 2 3

  phic = phi0 - path->getAng();
  giofile << phi0 << " " << pathAng << " " << phic << " ";  // using 4 5 6
  NORMALIZE_ANGLE(phic);

  u = this->u0;

  // get a value for omega from the control laws
  w = Compute_W(l, phic, this->a, u, &err);
  // limit the absolute angular velocity to avoid to big values for the motor
  double sign = w < 0 ? -1 : 1;
  w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign * (this->Vm / this->AXIS_LENGTH) : w;

  // w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? fabs(this->Vm / this->AXIS_LENGTH) : w;

  // transform the output of the Giovanni controller into input
  // for differential drive
  giofile << w << " ";
  vright = u - AXIS_LENGTH * w * 0.5;
  vleft = u + AXIS_LENGTH * w * 0.5;
  return true;
}
