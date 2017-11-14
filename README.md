# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Model Documentation

To simplify my model I used Frenet coordinates for planing. Method getXY provided (by instructor video) was not accurate enough. The car did not drive smoothly and had sudden jerks. Using spline function more accurate calculation of XY coordinates given Frenet coordinates is possible. Some inaccuracies are still present when calculating positions in the rightmost lane. This is because this lane is the farthest away from the reference line. I created additional class HighwayMap to encompass all map related functionalities.

##### States
Essentially, the model has two "states". The normal state is when I am considering all possible trajectories. That is, I am considering staying in the lane, or changing lane if possible. The second state is when I am changing lanes. In this state, I am reusing previously calculated waypoints and not considering new information. The only exception is when trajectory runs out of waypoints before reaching the targeted lane. In this case, I am generating all the trajectories like in the normal state. Obviously, this is a shortcoming of a model but makes the problem simpler, and it works well. This is the code in `main.cpp` that distinguishes two states:


```c++
if (in_lane || previous.size() < 20) { // "Normal" state - generate all trajectories 
  unordered_map<string, Trajectory> map = Trajectory::generate(previous, currentLocation, highwayMap, 4.0);
  unordered_map<string, double> costs;
  
  for (auto const& x : map) {
    costs[x.first] = x.second.cost(sensor_fusion);
  }
  state = minCostState(costs);

  selectedTrajectory = map[state];
}
else { // "Chaning lanes" state - resuse previously calculated trajectory
  selectedTrajectory = Trajectory::reuseTrajectory(previous, currentLocation, highwayMap);
}
```
After trying different values for trajectory length, i am using trajectories of 4.0 seconds duration. 

##### Generating trajectories
Static method `Trajectory::generate` generates all trajectories. If possible, first 10 waypoints of previously selected trajectory are reused. This makes the ride smoother. I am using `jmt` method to generate trajectories with a linear change in acceleration. 

```c++
vector<double> dif_acc = {-6.0, -4.0, -3.0, -2.0, -1.5, -1.0, -0.8, -0.6, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0};
vector<int> dif_lane = {0, 1, -1};
```
In code snippet above first vector contains all considered changes in acceleration (in seconds). This design choice simplifies model, by decrising number of parametes to tune. Second vector contains all considered changes in lanes. 

Trajectories are generated using `jmt` function in helper_functions.cpp. This is polynomial of 5th degree calculated such that it matches position, velocity and acceleration in starting and ending point of the trajectory. The same function is used in lectures.
Desired ening position is selected as :

```c++
// vector<double> s_start = {start_s, start_sv, start_sa}; <-- starting position
double dif_a = dif_acc[i];

double finish_a = start_sa + dif_a*T;
double finish_v = start_sv + start_sa*T + dif_a*T*T/2.0;
double finish_s = start_s + start_sv*T + start_sa*T*T/2.0 + dif_a*T*T*T/6.0;
```
Picture bellow depicts generated trajectry for linear chenge in acceleration of 0.3.
![Figure 1](imgs/figure_1.png?raw=true "Changes in s coordinate for linear increase in acceleration of 0.3")

##### Cost functions
Generated trajectories are compared according to cost functions implemented in cost_functions.cpp. All cost functions are normalized to the range 0.0 - 1.0. This makes comparing them somewhat easier. These values are then multiplied by matching cost coefficients. 
Functions are coeffiecients are given bellow
```c++
double Trajectory::cost (vector<vector<double>> sensor_fusion) const {
  double result = 0.0;

  double cost = collision_cost(sensor_fusion);
  result += cost*COLLISION;
  
  cost = inefficiency_cost();
  result += cost*EFFICIENCY;
  
  cost = acceleration_cost();
  result += cost*PROJECT_GOAL;
  
  cost = jerk_cost();
  result += cost*PROJECT_GOAL;
  
  cost = overspeeding_cost();
  result += cost*PROJECT_GOAL;

  cost = comfort_cost();
  result += cost*COMFORT;

  cost = change_lane_cost();
  result += cost*LANE_CHANGE; 

  cost = cost_outside_lanes();
  result += cost*TRAFIC_LAWS;
  
  return result;
}
```

```c++
const static double COLLISION = 1.0E6;
const static double PROJECT_GOAL = 1.0E6;
const static double TRAFIC_LAWS = 1.0E5;
const static double REACH_GOAL = 1.0E5;
const static double COMFORT = 1.0E4;
const static double EFFICIENCY = 3.0E2;
const static double LANE_CHANGE = 1.0E0;
```
Tunning these and parameters related with collision were realy time consuming. 

##### Shortcomings of the model

The implemented model does not estimate XY coordinates in rightmost lane given Frenet coordinates accurately enough. The consequence is, that sometimes the car exceeds the speed limit or moves outside right line. I countered this behavior by introducing speed limit buffer of 2.5 mi/h, but this also decreases overall car speed. To move car toward the center of the lane I introduced `cost_outside_lanes` cost function that penalizes trajectories that are closer to the edge of the road. 
Another problem area is when changing lanes. If some new information is obtained during this period (of about 3 seconds) my model does take them into account. This can be a real problem in realistic situations but not in this simulation environment.

Also, in my model, other vehicles are treated as they would never change lanes. Change lane behavior should be detected by continuously observing position and velocity of cars. This would mean more accurate implementation of getFrenet function.

Further work should eliminate these shortcomings.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

