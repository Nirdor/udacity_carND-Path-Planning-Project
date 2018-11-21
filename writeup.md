# **Path-Planning** 

---

**Path-Planning-Project**

##Goals:

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway.
The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

[//]: # (Image References)

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. Code must compile without errors with `cmake` and `make`.

My project includes the following files:
* src/MapTools.h header file for map related helper Functions
* src/MapTools.cpp source file for the map related helper Functions
* src/Planner.h header file for the trajectory planner
* src/Planner.cpp source file for the trajectory planner
* src/spline.h the spline function from the Tips section
* src/json.hpp library for handling json
* src/main.cpp main source file containig the main loop

I added the source files to the CMakeFiles.txt so everything should compile fine.

### Valid Trajectories 

#### The car is able to drive at least 4.32 miles without incident..

My car drives a lot more than 4.32 miles without incident...

#### The car drives according to the speed limit.

My planner tries to plan trajectories with a speed of 22.352m/s which is 1m/s slower than the allowed speed.
Because planning is done using frenet coordinates, the resulting speed after transforming to the map coordinating system can be faster or slower depending on the curvature of the road.
But it never exceeds the speed limit.

#### Max Acceleration and Jerk are not Exceeded.

My planner uses the quintic polynomial solver from the lesson to plan jerk minimzing trajectories. Acceleration and Jerk are not exceeded.

#### Car does not have collisions.

My planner checks every trajectory that is calculated from the solver for possible collisions and does not use them if any seem to may be the case.

#### The car stays in its lane, except for the time between changing lanes.

My planner plans the trajectory inside the lane if it does not want to change.

#### The car is able to change lanes

My planner can change the lane smoothly if it thinks that an other lane will be faster.

### Reflection

#### There is a reflection on how to generate paths.

see (Model Documentation.md)