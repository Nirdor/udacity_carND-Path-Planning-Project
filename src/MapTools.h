/*
 * MapTools.h
 *
 *  Created on: 16.10.2018
 *      Author: Martin
 */

#ifndef SRC_MAPTOOLS_H_
#define SRC_MAPTOOLS_H_

#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "spline.h"

using namespace std;

namespace planning_project {

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

struct PointXY {
  double x = 0.0;
  double y = 0.0;
  PointXY(){};
  PointXY(double x, double y)
    : x(x), y(y) {}
};

inline ostream& operator<<(ostream &o, const PointXY& pxy) {
  return o << "XY(" << pxy.x << ", " << pxy.y << ")";
}

struct PointSD {
  double s = 0.0;
  double d = 0.0;
  PointSD(){};
  PointSD(double s, double d)
    : s(s), d(d) {}
};

inline ostream& operator<<(ostream &o, const PointSD& psd) {
  return o << "SD(" << psd.s << ", " << psd.d << ")";
}

constexpr double l2norm(double a, double b) {
  return sqrt(a * a + b * b);
}
inline double l2norm(PointSD p) {
  return l2norm(p.s, p.d);
}
inline double l2norm(PointXY p) {
  return l2norm(p.x, p.y);
}

inline double normS(double s) {
  while (s < 0.0) {
    s += max_s;
  }
  while (s > max_s) {
    s -= max_s;
  }
  return s;
}

//compare a s-Coordinate with an other including the max_s so that 20 > 6000.
inline bool sGreaterThanS(double s1, double s2) {
  if (s1 > s2) {
    if (s1 - s2 > max_s / 2) {
      return false;
    }
    return true;
  } else if (s2 > s1) {
    if (s2 - s1 > max_s / 2) {
      return true;
    }
    return false;
  }
  return false;
}


//Use splines to convert from SD to XyY for a Smoother solution
class MapTools {

  tk::spline s_x;
  tk::spline s_y;
  tk::spline s_dx;
  tk::spline s_dy;

public:
  explicit MapTools(string map_file);
	virtual ~MapTools();

	PointXY getXY(PointSD sd);
};

} /* namespace planning_project */

#endif /* SRC_MAPTOOLS_H_ */
