/*
 * MapTools.cpp
 *
 *  Created on: 16.10.2018
 *      Author: Martin
 */

#include "MapTools.h"

#include <limits>

namespace planning_project {

MapTools::MapTools(string map_file) {

  //Read the Map information

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  ifstream in_map_(map_file.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //Add the beginning points to the end and the end points in front of the beginning to overlap.

  for (size_t i = 1; i < 4; ++i) {
    map_waypoints_x.insert(map_waypoints_x.begin(), map_waypoints_x.at(map_waypoints_x.size() - i));
    map_waypoints_y.insert(map_waypoints_y.begin(), map_waypoints_y.at(map_waypoints_y.size() - i));
    map_waypoints_s.insert(map_waypoints_s.begin(), map_waypoints_s.at(map_waypoints_s.size() - i));
    map_waypoints_dx.insert(map_waypoints_dx.begin(), map_waypoints_dx.at(map_waypoints_dx.size() - i));
    map_waypoints_dy.insert(map_waypoints_dy.begin(), map_waypoints_dy.at(map_waypoints_dy.size() - i));
  }

  map_waypoints_x.insert(map_waypoints_x.end(), map_waypoints_x.begin() + 3, map_waypoints_x.begin() + 6);
  map_waypoints_y.insert(map_waypoints_y.end(), map_waypoints_y.begin() + 3, map_waypoints_y.begin() + 6);
  map_waypoints_s.insert(map_waypoints_s.end(), map_waypoints_s.begin() + 3, map_waypoints_s.begin() + 6);
  map_waypoints_dx.insert(map_waypoints_dx.end(), map_waypoints_dx.begin() + 3, map_waypoints_dx.begin() + 6);
  map_waypoints_dy.insert(map_waypoints_dy.end(), map_waypoints_dy.begin() + 3, map_waypoints_dy.begin() + 6);

  map_waypoints_s.at(0) -= max_s;
  map_waypoints_s.at(1) -= max_s;
  map_waypoints_s.at(2) -= max_s;
  map_waypoints_s.at(map_waypoints_s.size() - 3) += max_s;
  map_waypoints_s.at(map_waypoints_s.size() - 2) += max_s;
  map_waypoints_s.at(map_waypoints_s.size() - 1) += max_s;

  s_x.set_points(map_waypoints_s, map_waypoints_x);
  s_y.set_points(map_waypoints_s, map_waypoints_y);
  s_dx.set_points(map_waypoints_s, map_waypoints_dx);
  s_dy.set_points(map_waypoints_s, map_waypoints_dy);

}

MapTools::~MapTools() {
}

PointXY MapTools::getXY(PointSD sd) {
  double s = normS(sd.s);
  return PointXY(s_x(s) + sd.d * s_dx(s), s_y(s) + sd.d * s_dy(s));
}

} /* namespace planning_project */
