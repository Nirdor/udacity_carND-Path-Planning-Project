/*
 * Planner.cpp
 *
 *  Created on: 15.10.2018
 *      Author: Martin
 */

#include "Planner.h"

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace planning_project {

Planner::Planner(MapTools* tools) : tools(tools) {
}

Planner::~Planner() {
}

//Update status variables
void Planner::updateStatus(StatusInfo& status_info, vector<Sensor>& fusion_data, size_t points_walked) {
  status = status_info;
  this->points_walked = points_walked;

  current_lane = lane_of(status.sd.d);

  obstacles.clear();
  lane_change_left_possible = current_lane > Lane::L;
  lane_change_right_possible = current_lane < Lane::R;
  for (Sensor s : fusion_data) {
    //Check if Car is in an interesting area
    if (s.sd.d > 0.0 && s.sd.d < 3 * lane_width && sGreaterThanS(s.sd.s, status.sd.s - 10) && sGreaterThanS(status.sd.s + 100, s.sd.s)) {
      //Check for lane_switch
      Lane l = lane_of(s.sd.d);
      if (lane_change_left_possible && l == current_lane - 1 && sGreaterThanS(s.sd.s, status.sd.s - 8) && sGreaterThanS(status.sd.s + 8, s.sd.s)) {
        lane_change_left_possible = false;
      }
      if (lane_change_right_possible && l == current_lane + 1 && sGreaterThanS(s.sd.s, status.sd.s - 8) && sGreaterThanS(status.sd.s + 8, s.sd.s)) {
        lane_change_right_possible = false;
      }

      obstacles.push_back(Car(s.xy, s.speed, l, s.sd.s));
    }
  }

  cout << "Satus: " << status.sd << " :" << status.speed << endl;
  cout << "points_walked: " << points_walked << endl;
  cout << "Num Cars: " << obstacles.size() << endl;
}

//Main Trajectory Generation function
pair<vector<double>, vector<double>> Planner::generateTrajectory() {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  TrajPointSD start;
  double duration = horizon;
  if (trajectorySD.size() < points_walked + keep_points + 1) {
    start.sd = status.sd;
    start.speed.s = status.speed;
  } else {
    //Keep the first points identical to the last trajectory
    start = trajectorySD.at(points_walked + keep_points);
    duration = horizon - (keep_points * time_between_points);
  }

  //Generate possible target Points order from best to worst
  vector<Target> targets = generatePossibleTargets();

  //Calculate a trajectory using the quintic polynomial solver for each target and check for possible collisions
  //If no collisions are found use this trajectory
  vector<TrajPointSD> traj;
  bool found_traj = false;
  for (Target t : targets) {
    TrajPointSD target;
    target.sd = t.sd;
    target.speed.s = t.speed;

    traj = calculateTrajectorySD(start, target, duration);
    if (traj.size() == 0) {
      continue;
    }
    if (checkCollision(traj)) {
      found_traj = true;
      break;
    }
    cout << "Target with Collision: " << t.sd.s << ":" << t.sd.d << ":" << t.speed << endl;
  }

  if (!found_traj) { //If planning fails, just break
    TrajPointSD target;
    double a = -status.speed / horizon;
    double dist = max(status.speed * horizon + a / 2.0 * horizon * horizon, 0.0);
    target.sd = PointSD(status.sd.s + dist, status.sd.d);
    target.speed.s = 0.0;
    traj = calculateTrajectorySD(start, target, duration);
  }

  if (trajectorySD.size() > keep_points && (status.sd.s > 6940 || status.sd.s < 5)) {
    //forget Planning around max_s, it has probably Errors (TODO: find these and fix), just use the existing trajectory during this time
    traj.clear();
    traj.insert(traj.begin(), trajectorySD.begin() + points_walked, trajectorySD.end());
  } else if (trajectorySD.size() > points_walked + keep_points) {
    //Keep the first points identical to the last trajectory
    traj.insert(traj.begin(), trajectorySD.begin() + points_walked, trajectorySD.begin() + points_walked + keep_points);
  }
  trajectorySD = traj;

  //Convert from SD to XY and return
  for (TrajPointSD p : traj) {
    PointXY pxy = tools->getXY(p.sd);
    next_x_vals.push_back(pxy.x);
    next_y_vals.push_back(pxy.y);
  }

  return make_pair(next_x_vals, next_y_vals);
}

//Generate possible target Points order from best to worst
vector<Target> Planner::generatePossibleTargets() {

  vector<Target> ret;

  //Check for possible lanes to drive on
  vector<Lane> possible_lanes;
  switch (current_lane) {
  case Lane::L:
    possible_lanes.push_back(Lane::L);
    if (lane_change_right_possible) {
      possible_lanes.push_back(Lane::M);
    }
    break;
  case Lane::M:
    possible_lanes.push_back(Lane::M);
    if (lane_change_right_possible) {
      possible_lanes.push_back(Lane::R);
    }
    if (lane_change_left_possible) {
      possible_lanes.push_back(Lane::L);
    }
    break;
  case Lane::R:
    possible_lanes.push_back(Lane::R);
    if (lane_change_left_possible) {
      possible_lanes.push_back(Lane::M);
    }
    break;
  }

  //Find the lane with the most space to drive fast
  vector<double> possible_distance;
  for (Lane l : possible_lanes) {
    double max = 200.0;
    for (Car c : obstacles) {
      if (c.lane == l && c.s > status.sd.s && c.s - status.sd.s < max) {
        max = normS(c.s + l2norm(c.speed) * horizon - status.sd.s) - status.speed / 2;
      }
    }
    possible_distance.push_back(max);
  }

  double max_range = 0.0;
  size_t pos = 0;
  for (size_t i = 0; i < possible_lanes.size(); ++i) {
    if (possible_distance.at(i) > max_range) {
      pos = i;
      max_range = possible_distance.at(i);
    }
  }

  //Lane at pos is the best lane
  Lane l = possible_lanes.at(pos);
  for (int i = 0; i < 10; ++i) {
    double target_speed = max_speed - i * (max_speed / 9);
    if (l != current_lane) {
      target_speed *= 0.8;
    }
    double a = (target_speed - status.speed) / horizon;
    double dist = min(min(status.speed, target_speed) * horizon + a / 2.0 * horizon * horizon, max_range);
    double d = lane_width / 2.0 + (static_cast<std::underlying_type<Lane>::type>(l) + 1) * lane_width;
    if (l == Lane::R) { //On the right Lane subtract 0.12 because of outside lane bug
      d -= 0.12;
    }
    Target t = Target(PointSD(status.sd.s + dist, d), target_speed, horizon);
    ret.push_back(t);
  }
  //Add targets from other lanes too
  possible_lanes.erase(possible_lanes.begin() + pos);
  for (Lane l : possible_lanes) {
    for (int i = 0; i < 10; ++i) {
      double target_speed = max_speed - i * (max_speed / 9);
      if (l != current_lane) {
        target_speed *= 0.8;
      }
      double a = (target_speed - status.speed) / horizon;
      double dist = min(min(status.speed, target_speed) * horizon + a / 2.0 * horizon * horizon, max_range);
      double d = lane_width / 2.0 + (static_cast<std::underlying_type<Lane>::type>(l) + 1) * lane_width;
      if (l == Lane::R) {
        d -= 0.12;
      }
      Target t = Target(PointSD(normS(status.sd.s + dist), d), target_speed, horizon);
      ret.push_back(t);
    }
  }

  return ret;
}

//use the Quintic Polynomial solver from the lesson to generate a trajectory
vector<TrajPointSD> Planner::calculateTrajectorySD(TrajPointSD start, TrajPointSD target, double duration) {

  vector<TrajPointSD> ret;

  double s0, s1, s2, d0, d1, d2, T;

  s0 = start.sd.s;
  s1 = start.speed.s;
  s2 = start.acc.s / 2;

  d0 = start.sd.d;
  d1 = start.speed.d;
  d2 = start.acc.d / 2;

  T = duration;

  double T2 = T * T;
  double T3 = T2 * T;
  double T4 = T3 * T;
  double T5 = T4 * T;

  MatrixXd A(3, 3);
  A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;
  VectorXd a(3), b(3);
  a << target.sd.s - (s0 + s1 * T + s2 * T2), target.speed.s - (s1 + 2 * s2 * T), target.acc.s - s2 * 2;
  b << target.sd.d - (d0 + d1 * T + d2 * T2), target.speed.d - (d1 + 2 * d2 * T), target.acc.d - d2 * 2;
  VectorXd s = A.inverse() * a;
  VectorXd d = A.inverse() * b;


  for (double i = 0.0; i <= T; i += time_between_points) {
    double ps = s0 + s1 * i + s2 * i * i + s[0] * i * i * i + s[1] * i * i * i * i + s[2] * i * i * i * i * i;
    double speed = s1 + 2 * s2 * i + 3 * s[0] * i * i + 4 * s[1] * i * i * i + 5 * s[2] * i * i * i * i;
    double acc = 2 * s2 + 6 * s[0] * i + 12 * s[1] * i * i + 20 * s[2] * i * i * i;
    double pd = d0 + d1 * i + d2 * i * i + d[0] * i * i * i + d[1] * i * i * i * i + d[2] * i * i * i * i * i;
    double dspeed = d1 + 2 * d2 * i + 3 * d[0] * i * i + 4 * d[1] * i * i * i + 5 * d[2] * i * i * i * i;
    double dacc = 2 * d2 + 6 * d[0] * i + 12 * d[1] * i * i + 20 * d[2] * i * i * i;
    if (l2norm(speed, dspeed) > max_speed || l2norm(acc, dacc) > max_acc || speed < 0) {
      return vector<TrajPointSD>();
    }
    ret.push_back(TrajPointSD(PointSD(normS(ps), pd), PointSD(speed, dspeed), PointSD(acc, dacc)));
  }

  return ret;
}

//Check target trajectory for possible collisions
bool Planner::checkCollision(vector<TrajPointSD> traj) {

  for (size_t i = 0; i < traj.size(); i += 2) { //Check only every second point for speed
    //Transform each point into the local coordinate system of the obstacle car and check if it lies inside the bounds.
    PointXY xy = tools->getXY(traj.at(i).sd);
    for (Car c : obstacles) {
      double car_x = c.xy.x + i * time_between_points * c.speed.x;
      double car_y = c.xy.y + i * time_between_points * c.speed.y;
      double transformed_x = (xy.x - car_x) * cos(c.yaw) + (xy.y - car_y) * sin(c.yaw);
      double transformed_y = (xy.x - car_x) * -sin(c.yaw) + (xy.y - car_y) * cos(c.yaw);
      if (transformed_x > -2.5 && transformed_x < 2.5 && transformed_y > -1 && transformed_y < 1) {
        return false;
      }
    }
  }

  return true;
}

} /* namespace planning_project */
