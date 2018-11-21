/*
 * Planner.h
 *
 *  Created on: 15.10.2018
 *      Author: Martin
 */

#ifndef SRC_PLANNER_H_
#define SRC_PLANNER_H_

#include <utility>
#include <vector>
#include <iostream>

#include "MapTools.h"

using namespace std;

namespace planning_project {

enum class Lane : int {L = -1, M = 0, R = 1};
constexpr double lane_width = 4.0;

constexpr Lane lane_of(double d) {
  return Lane(static_cast<underlying_type<Lane>::type>(d / lane_width) - 1);
}

inline Lane operator-(Lane a, int b) {
  return Lane(static_cast<underlying_type<Lane>::type>(a) - b);
}
inline Lane operator+(Lane a, int b) {
  return Lane(static_cast<underlying_type<Lane>::type>(a) + b);
}

struct StatusInfo {
  PointXY xy;
  PointSD sd;
  double yaw = 0.0;
  double speed = 0.0;
  StatusInfo(){};
  StatusInfo(double x, double y, double s, double d, double yaw, double speed)
    : xy(x, y), sd(s, d), yaw(yaw), speed(speed) {};
};

struct Sensor {
  int id;
  PointXY xy;
  PointXY speed;
  PointSD sd;
  Sensor(int id, double x, double y, double speed_x, double speed_y, double s, double d)
    : id(id), xy(x, y), speed(speed_x, speed_y), sd(s, d) {}
};

struct Car {
  PointXY xy;
  PointXY speed;
  double yaw;
  Lane lane;
  double s;
  Car(PointXY xy, PointXY speed, Lane lane, double s)
    : xy(xy), speed(speed), lane(lane), s(s) {
    yaw = atan2(speed.y, speed.x);
  }
};

struct Target {
  PointSD sd;
  double speed = 0.0;
  double duration = 0.0;
  Target() {};
  Target(PointSD sd, double speed, double duration)
    : sd(sd), speed(speed), duration(duration) {};
};

struct TrajPointSD {
  PointSD sd;
  PointSD speed;
  PointSD acc;
  TrajPointSD() {};
  TrajPointSD(PointSD sd, PointSD speed, PointSD acc)
    : sd(sd), speed(speed), acc(acc) {};
};

struct TrajPointXY {
  PointXY xy;
  PointXY speed;
  PointXY acc;
  TrajPointXY() {};
  TrajPointXY(PointXY xy, PointXY speed, PointXY acc)
    : xy(xy), speed(speed), acc(acc) {};
};

const double time_between_points = 0.02;
const double points_per_sec = 1.0 / time_between_points;
const size_t keep_points = 10;
const double max_speed = 22.352; //23.352;
const double max_acc = 10.0; //23.352;
const double horizon = 2.5;

class Planner {

  StatusInfo status;
  Lane current_lane = Lane::M;
  MapTools* tools;
  vector<TrajPointSD> trajectorySD;
  vector<TrajPointXY> trajectoryXY;
  size_t points_walked;
  vector<Car> obstacles;
  bool lane_change_left_possible = false;
  bool lane_change_right_possible = false;

public:
	Planner(MapTools* tools);
	virtual ~Planner();

	void updateStatus(StatusInfo& status_info, vector<Sensor>& fusion_data, size_t points_walked);
	pair<vector<double>, vector<double>> generateTrajectory();

private:
	vector<Target> generatePossibleTargets();
	vector<TrajPointSD> calculateTrajectorySD(TrajPointSD start, TrajPointSD target, double duration); //Quintic Polynomial Solver
	bool checkCollision(vector<TrajPointSD> traj);

};

} /* namespace planning_project */

#endif /* SRC_PLANNER_H_ */
