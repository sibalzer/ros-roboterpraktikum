#ifndef __MY_CURVES_H
#define __MY_CURVES_H__

#include <math.h>
#include <ros/ros.h>

#include <fstream>
#include <iostream>

#define CN_NOT_ACT -2

using namespace std;

class CCurve
{
protected:
  /**
   * An array that holds the x coordinate of all drive points.
   */
  double* tx;
  /**
   * An array that holds the y coordinate of all drive points.
   */
  double* ty;

  double A, B, C;

  /**
   * The number of points in the point list.
   */
  int count;
  /**
   * The index of the current point in the point list.
   */
  int cur;
  int add_c;
  /**
   * Is not zero, if an error during initialization happened.
   */
  int c_err;

  inline void updateCoefficients()
  {
    A = ty[cur + 1] - ty[cur];
    B = tx[cur] - tx[cur + 1];
    C = -A * tx[cur] - B * ty[cur];
  }

public:
  /**
   * Creates a new curves object with a given number of points.
   * @param num the number of points on the curve. Defaults to {@code 0}
   */
  CCurve(int num = 0);

  ~CCurve();

  /**
   * Writes the loaded points back to the given file.
   * First, it puts the number of points in the first line.
   * Then, for every point, it writes the x and y coordinate
   * separated by a whitespace on a line for each point.
   *
   * @param fname the path to the file to write to
   * @returns {@code true} when the file was successfully written
   */
  bool WriteToFile(const char* fname);

  /**
   * Reads the contents from the given file.
   * It parses the number of points
   * and the x and y coordinates from every point in the list.
   *
   * @param fname the path to the file to read from
   * @returns {@code true} when the file was successfully read
   */
  bool LoadFromFile(const char* fname);

  double evaluate(const double x, const double y) const;

  double getAng();

  double getTan();

  /**
   * Resets the current driving point to the initial point.
   *
   * @returns {@code true} when the driving point was successfully reset
   */
  bool initTraversal();

  bool getNext(int looped = 0);

  bool addNextPoint(double x, double y);

  inline int getSegmentNumber() const
  {
    if (c_err)
    {
      return -1;
    }

    return cur;
  }

  /**
   * Checks, if the given coordinates
   * are orthogonal to the current path segment.
   * 
   * @param x the current x coordinate of the robot
   * @param y the current y coordinate of the robot
   */
  bool pointIn(double x, double y) const;

  /**
   * Returns the distance between the nearest point on the linear path
   * between the current and next point in the point list.
   *
   * @param x the current x coordinate of the robot
   * @param y the current y coordinate of the robot
   * @returns the distance from the linear path
   */
  double getDistance(double x, double y) const;

  /**
   * Returns the number of points in the point list
   * @returns the number of points
   */
  inline int getCount() const
  {
    return this->count;
  }

  /**
   * Returns the distance to the destination point on the curve.
   *
   * @param x the current x coordinate of the robot
   * @param y the current y coordinate of the robot
   * @returns the distance to the destination point
   */
  double getDistanceToEnd(double x, double y) const;
};

#endif
