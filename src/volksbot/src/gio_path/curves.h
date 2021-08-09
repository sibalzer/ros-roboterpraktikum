#ifndef __MY_CURVES_H
#define __MY_CURVES_H__

#include <math.h>
#include <iostream>
#include <fstream>

#define CN_NOT_ACT -2

using namespace std;

class CCurve
{
protected:
  double* tx;
  double* ty;
  double A, B, C;

  int count;
  int cur;
  int add_c;
  int c_err;

  inline void recompute_coeffs(int i)
  {
    A = ty[i + 1] - ty[i];
    B = tx[i] - tx[i + 1];
    C = -A * tx[i] - B * ty[i];
  }

public:
  CCurve(int num = 0);

  ~CCurve();

  int WriteToFile(const char* fname);

  int LoadFromFile(const char* fname);

  double Evaluate(double x, double y);

  double getAng();

  double getTan();

  int initTraversal();

  int getNext(int looped = 0);

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
