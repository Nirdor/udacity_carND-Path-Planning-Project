# **Path-Planning** 

---

**Path-Planning-Project**

##Model Documentation:

The planning model follows these steps to generate a path that is send back to the simulator.

1. Update the internal status with the data from the simulator. This is implemented in the function `Planner::updateStatus`. It uses the following importand information:
  * The current Position in SD coordinates, this is immediately used to calculate the current lane (Left, Middle or Right)
  * The current speed
  * The number of points that the simulator traveled from the last path
  * The sensor fusion data containig the obstacle cars. This data is immediately filtered for interesting obstacles and used to calculate if a lane change is currently possible.
  
2. Call the function `Planner::generateTrajectory()` which generates the final Path. It uses several helper functions to do so. First, it keeps 10 Points from the last path and use the 11th point as start for new planning.

3. Generate possible target points for the solver. This is implemented in the helper function `Planner::generatePossibleTargets`.
Target points are generated on the current lane and if a lane change is possible, on the corresponding lanes. Target points are generated in different distances resulting in different speeds.
The farthest point aims at the target speed of 22.352m/s. The resulting points are ordered that the farthest on the most interesting lane come first.

4. For each target point use the quintic polynomial solver to calculate the resulting path (`Planner::calculateTrajectorySD`). Then check this path for possible collisions(`Planner::checkCollision`).
If it is collision free use this path. In the rare case that no collision free path can be found using this method the planner defaults to a breaking path resulting the speed to zero on the current lane.

5. Add the kept points from the last path to the trajectory and convert it to the map XY coordinates using a spline for smooth transition.

Finally hand it back to the simulator.