# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Problem Statement

Implement an Extended Kalman filter to track a bicycle moving around the car. Using radar and laser measurements develop a sensor fusion algorithm in C++ to track the bicycle's position and velocity.

## Algorithm flow

The tracking is achieved using an Extended Kalman filter. The main steps in programming a Kalman filter are:

1. **Initializing** Kalman filter variables
2. **Predicting** where the object is going to be after a time step Δt
3. **Updating** where the object is based on sensor measurements

The 2 measurements - laser and radar are used to track the bicycle using a Kalman filter and Extended Kalman filter respectively. The process flow is summarized in the diagram below.

 ![Alt text](EKF-flow.png?raw=true)

## Output

The tracking accuracy is measured using the RMSE metric for position and velocity in x and y directions.

For sample-laser-radar-measurement-data-1.txt, the RMSE values are:

| Measurement | RMSE|
| --- | --- |
| Position (x) | 0.0664081 |
| Position (y) | 0.0599689 |
| Velocity (x) | 0.505135  |
| Velocity (y) | 0.528294  |


The following is a visualization of the tracking vs the actual position of the object:

![Alt text](EKF_visualization1.png?raw=true)

For sample-laser-radar-measurement-data-2.txt, the RMSE values are:

| Measurement | RMSE|
| --- | --- |
| Position (x) | 0.176539 |
| Position (y) | 0.187801 |
| Velocity (x) | 0.425854  |
| Velocity (y) | 0.731882  |

The following is a visualization of the tracking vs the actual position of the object:

![Alt text](EKF_visualization2.png?raw=true)

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt out1.txt`
    or `./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt out2.txt`
