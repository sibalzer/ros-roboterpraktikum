#ifndef __MY_CURVES_H
#define __MY_CURVES_H__

#include <math.h>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

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
  /**
   * Matrix coefficients.
   */
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

  inline void recompute_coeffs(int i)
  {
    A = ty[i + 1] - ty[i];
    B = tx[i] - tx[i + 1];
    C = -A * tx[i] - B * ty[i];
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

  double Evaluate(double x, double y);

  double getAng();

  double getTan();

  /**
   * Resets the current driving point to the initial point.
   * 
   * @returns {@code true} when the driving point was successfully reset
   */
  bool initTraversal();

  /**
   * Determines the next index 
   */
  bool getNext(int looped = 0);

  int getPrev(int looped = 0);

  int addNextPoint(double x, double y);

  int getSegmentNumber();

  int pointInn(double x, double y);

  double getDistance(double x, double y);

  inline int getCount()
  {
    return this->count;
  }

  double getDistanceToEnd(double x, double y);
};

#endif
