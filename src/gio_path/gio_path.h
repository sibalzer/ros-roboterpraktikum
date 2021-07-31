/**
 *  Implementation of a Giovanni Controller for mobile robots.
 *  As reference, the paper SWITCHING LINEAR PATH FOLLOWING FOR
 *  BOUNDED CURVATURE CAR-LIKE VEHICLE by Giovanni Idiveri, 2004 
 *  can be seen.
 */

#ifndef __GIO_PATH_H_
#define __GIO_PATH_H_

#include <math.h>
#include "curves.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>

using namespace std;

#define d_y_0               0.0002         //ab wann y als 0 wahrgenommen werden kann 
#define d_th_0              0.001          //ab wann th als 0 wahrgenommen werden kann

#define AC_GRAD             0
#define AC_RAD              1

#define ABS                 1
#define RES                 0

#define GradToRadian        0.01745328
#define RadianToGrad       57.295827909

#define NormalizeAngle(ang) while(fabs(ang)>M_PI) ang+=(ang>0)?-2*M_PI:2*M_PI
#define SQR(c)  ((c) * (c))
/**
 * @class CGioController
 * @brief class encapsules path follower
 * @author Niko, Hartmut, Stefan
 * @see code fragment for using
 */
class CGioController{
protected:
  /**
   * ...
   */
  CCurve *path;
  int loop_exit;
  // local coordinate system of the robot
  double ex[2]; 
  double ey[2];

  double x0; // initial pose?
  double y0;
  double phi0;

  // found coordinates of the robot
  double AXIS_LENGTH; // 2b
  double Vm;   // limit for motor speed         
  double d_y;  // delta y from the paper, minimal value to evaluate     
  double d_th; // delta theta from the paper, minimal value to evaluate         
  double kr_max; // maximal curvature
  double u0;     // initial velocity
  double a;        // gain alpha
  double epsilon; // to treat values near 0 as 0

  /**
   *  ...
   */  
  void InitDefault();
  /**
   * Method to set local coordinate system
   * @param ang angle for rotating the coordinate system
   */
  void setLocalSystem(double ang);
  
  /**
   *   Calculate h_j* according to eq 23 in the paper
   *   Calculate gamma_j according to eq 26 in the paper
   *
   *   @param y coordinate of the robot w.r.t. goal
   *   @param th angle of the pose (theta) w.r.t. goal
   *   @param u forward speed
   *   @param alpha gain, >1
   *   @param gama gamma, gain of the control law
   *   @return h gain of the controller
   */
  double H_case_1(double y, double th, double u, double alpha, double *gama);
  /**
   *   Calculate h_j* according to eq 24 in the paper
   *   Calculate gamma_j according to eq 26 in the paper
   *   @param y coordinate of the robot w.r.t goal
   *   @param th angle of the pose (theta) w.r.t. goal
   *   @param u forward speed
   *   @param alpha gain, >1
   *   @param gama gamma, gain of the control law
   *   @return h gain of the controller
   */
  double H_case_2(double y, double th, double u, double alpha, double *gama);
  /**
   *  compute the rotation rate required
   *  @param y coordinate of the robot w.r.t. goal
   *  @param th angle of the pose (theta) w.r.t. goal
   *  @param a alpha, gain >1
   *  @param u forward speed
   *  @param err error handling
   *  @return omega (rotation rate)
   */
  double Compute_W(double y, double th, double a, double u, int *err);

public:
  std::ofstream giofile; // file to log to
  
  /**
   *  Construct a new instance of the Giovanni Controller
   */
  CGioController();

  /**
   *  Destruct the instance
   */
  ~CGioController();
    
  /**
   *  Configurate the axis length 
   *  @param val new value in [m?]
   */
  void setAxisLength(double val);
  /**
   *  Get the current value for the axis length
   *  @return axis length in [m?]
   */
  double getAxisLength();	

  /**
   *  Set the velocity to a specific value
   *  @param val
   *  @param abs
   */
  void setCurrentVelocity(double val, int abs = ABS);

  /**
   *  Get the current velocity
   *  @return velocity in [m/s?]
   */
  double getCurrentVelocity();
  
  /**
   *  
   */
  void setPose(double x, double y, double phi);
  void getPose(double &x, double &y, double &ph);
  int getPathFromFile(const char* fname);
  int writePathToFile(const char* fname);
  int canDetermineRobotPosition(int looped = 0);
  int getNextState(double &u, double &w, double &vleft, double &vright, int looped = 0);
};

#endif
