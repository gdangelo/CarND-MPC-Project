# CarND-MPC-Project

> Model Predictive Control (MPC) Project for Self-Driving Car ND

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![MPC Controller]()

## Project Basics

The goal of this project is to implement in C++ a Model Predictive Controller (MPC) that can drive a car around a [virtual track](https://github.com/udacity/self-driving-car-sim/releases) using specific waypoints from the track itself. The MPC will predict each actuator to control the steering angle and the throttle of the car along the track. The car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

**Project Steps**

- Fitting a polynomial based on road waypoints 
- Evaluating the current state of the car (position, bearing, errors) based on the polynomial
- Implementing the MPC calculations by settings the timestep, number of steps, variables and constraints
- Defining steering angle and throtlle based on MPC results
- Taking into account car's actuators latency (100ms)
- Testing and tuning the implementation on Udacity simulator

**Results**



## Discussion / Reflection



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.



## Basic Build Instructions
Starting to work on this project consists of the following steps:

1. Install all the required [dependencies](#dependencies)
2. Clone this repository
3. Build the main program 
    - `mkdir build`
    - `cd build`
    - `cmake ..`
    - `make`
4. Run `./mpc`
5. Launch the Udacity Term 2 simulator
6. Enjoy!


## Questions or Feedback

> Contact me anytime for anything about my projects or machine learning in general. I'd be happy to help you :wink:

* Twitter: [@gdangel0](https://twitter.com/gdangel0)
* Linkedin: [Grégory D'Angelo](https://www.linkedin.com/in/gregorydangelo)
* Email: [gregory@gdangelo.fr](mailto:gregory@gdangelo.fr)
