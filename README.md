# CarND-Path-Planning-Project

This project implements a _Navigator_ (composed of a _Behavior Planner_ and a _Path Planner_) to create smooth, safe paths for an autonomous car to drive along. It communicates with Udacity's simulator through a WebSocket interface, receiving as input the car state, obstacle data (i.e. data on other vehicles) and current path plan, and sending back a new sequence of waypoints describing the updated path plan. The diagram below summarizes the system architecture:

<img src="https://xperroni.github.io/CarND-Path-Planning-Project/architecture.jpg">

The WebSocket interface is implemented in file [src/main.cpp](src/main.cpp). It encapsulates a [Navigator](src/navigator.h) object, which by its turn encapsulates instances of [BehaviorPlanner](src/behavior_planner.h) and [PathPlanner](src/path_planner.h). `Navigator` and `BehaviorPlanner` together decode the WebSocket inputs to collect the current path plan, composed of a sequence of waypoints in global Cartesian coordinates `(x, y)`; the current car [State](src/state.h), composed of global Cartesian coordinates `(x, y)`, orientation `o`, Frenet coordinates `(s, d)`, speed `v` and current `lane` index; and [Obstacles](src/obstacles.h) data, containing the Cartesian and Frenet coordinates, speeds and lane index of all vehicles on the same side of the road as the car. A [HighwayMap](src/highway_map.h) is used to keep track of all vehicles along the highway. Updated path plans are sent back to the simulator through the WebSocket interface.

Because the Path Planner must react quickly to changing road conditions, the plan covers a period of only 0.3<i>s</i>; each turn the first 0.1<i>s</i> worth of waypoints from the current path plan is kept, and a new plan is constructed from the last kept waypoint onwards. Accordingly, the first step of the planning process is to update `State` and `Obstacles` objects to reflect expected conditions 0.1<i>s</i> into the future. The `BehaviorPlanner` then updates its internal Finite State Machine (FSM) according to predicted readings and its own rules, as illustrated below:

<img src="https://xperroni.github.io/CarND-Path-Planning-Project/behavior_fsm.jpg">

The FSM starts at the `START` state, which determines the initial lane and the switches into the `CRUISING` state. In this state the car moves at the reference speed of 20<i>m/s</i> (close to 44MPH); it also tries to keep the car on the middle lane, from where it can more easily move to avoid slower cars. If it finds a slower car ahead, it initially switches to the `TAILING` state where it will try to pair its speed to them, while watching for an opportunity to change lanes. When this comes the FSM selects a destination lane and switches to `CHANGING_LANES` state, returning to `CRUISING` state once the movement is complete.

Once the current behavior (composed of a reference speed `v` and a polynomial `route` roughly indicating whether to keep or change lanes) is determined, it's dispatched along the current `State` to the `PathPlanner`. It in turn uses the [CppAD interface to Ipopt](https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm) to compute a sequence of waypoints approaching the route at the reference speed, while respecting acceleration limits.

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
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. Find the latest version [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

On BASH-enabled environments you may also run the `build.sh` script to build the project and create a symbolic link to the executable in the base project directory.
