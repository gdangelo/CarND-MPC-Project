# CarND-MPC-Project

> Model Predictive Control (MPC) Project for Self-Driving Car ND

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![MPC Controller](https://user-images.githubusercontent.com/4352286/37614999-92f583c0-2b82-11e8-8573-72aed61c0b94.png)

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

See video of the results from my implementation [here](https://github.com/gdangelo/CarND-MPC-Project/blob/master/MPC.mov).

## Discussion / Reflection

**The Model**

My MPC model follows the one from the Udacity lessons. The state updates are defined by the following equations:

![MPC Model](https://user-images.githubusercontent.com/4352286/37605262-316d5b8c-2b69-11e8-9436-c3f5ec897876.png)

The actuators values are also constrained by the simulator limits for the steering angle (delta) and the throttle (a).

The cost function that the MPC solver need to minimize is composed of three main parts (see lines 60-85 of `MPC.cpp`):

- Cost related to the reference state: I penalize for cross-track error, psi error, and reference speed gap (reference speed is set to 90mph).
- Cost related to the use of actuators: I penalize for using too much steer and throttle, but also when accelerating too much on sharp turns.
- Cost related to sequential acutations: I penalize for hard change in steering angle and throttle values. It helps smooth the driving experience.

Each cost is multiplied by a different weight before summing them up all together. These costs have been tuned manually to give more importance to certain costs than others (see lines 36-44 of `MPC.cpp`). 

For a detailed implementation of my MPC model and calculations, please see the `Solve()` function and the `FG_eval` class of the `MPC.cpp` file.

**Timestep Length and Elapsed Duration (N & dt)**

Timestep length (N) and elapsed duration (dt) used by the MPC solver have been tuned manually. After several tests, the final values have been defined to **N = 9**, and **dt = 0.14**. I was originally using N = 10 and dt = 0.1 because I've thought that 1s would be a good prediction window. However, the results were not satisfying. So, I tried to find a larger prediction window size without increasing dt too much. It turns out that the final values chosen were good enough to have the car performs its task at a high speed and without shaking too much.

**Polynomial Fitting and Initial State**

First, I transformed the waypoints from the system's global coordinates to the car's coordinates. See `main.cpp` from line 97 to line 105. The following equations were used to perform the transformation:

- `waypoints_x = dx * cos(-psi) - dy * sin(-psi)`
- `waypoints_y = dx * sin(-psi) + dy * cos(-psi)`

where:

- `dx = ptsx[i] - px`, is the difference between the waypoint x-position `ptsx` and the current car x-position `px` in global coordinates
- `dy = ptsy[i] - py`, is the difference between the waypoint y-position `ptsy` and the current car y-position `py` in global coordinates
- `psi`, is the current orientation angle of the car

Then, using the `polyfit()` function (see lines 107-122 of `main.cpp`), a third order polynomial line is fitted from the transformed waypoints. This line represents the path that the car should follow when driving.

Finally, from the above steps, the initial/current state of the car could be defined in order to feed the MPC solver and find the best actuators values. Before taking latency into account, the initial state (see lines 114-120 of `main.cpp`) was defined as folllow:  `px = 0, py = 0, and psi = 0`. Indeed, because we are operating from the car's coordinates, we can set px and py equal to zero since the car is the origin of the coordinate system. Furthermore, psi can also be set to zero because the x-axis always points in the direction of the car's heading.

Regarding the cross-track-error, as the car's position (px) is zero, it is calculated by evaluating the polynomial function at zero (see lines 119 of `main.cpp`). The psi error, or epsi, is calculated from the derivative of the polynomial fit line at px which is zero. So, the calculation is simplified to the negative arc tangent of the second coefficient of the polynomial (see lines 120 of `main.cpp`).

**Model Predictive Control with Latency**

The system suffers from a 100ms latency to perform the action when best actuators values have been calculated by MPC. Hence, in order to compensate this delay, I predicted and set the initial state of the car 100ms later before performing the MPC calculations. This is done using the same update equations used in my MPC model (see lines 122-130 of `main.cpp`). Note that these equations are simplified due to the coordinate system transformation.

So, this initial predicted state and the third-order polynomial line are then fed to the MPC solver to find the best actuators values.

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
