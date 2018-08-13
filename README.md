# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
This is my solution for the model predictive controller project for Udacity's CarND program.

---

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

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation Description
### The Model
The model used is a kinematic model defined by the state equations below:

      x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      v[t] = v[t-1] + a[t-1] * dt
      cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
 
 Where:
 * x, y - X & Y Coordinates
 * psi  - The car's heading
 * v    - The car's velocity
 * cte  - The cross-track error (distance from the middle of the road)
 * epsi - Orientation error
 
 Lf is the distance from the front axle to the center of gravity of the car. The model outputs are the accleration and steering:
       a     - accleration
       delta - steering
 
### Timestep Length and elapsed duration (N & dt)
The timestep length and elapsed duration (N & dt) are chosen as 10 and 0.1 respectively. This would result in the optimizer considering a duration of 1 second (10 * 0.1 = 1 s) to determine a trajectory. The timestep length and elapsed duration should be small enough to allow for quick computation (without burdening the on-board computer), but large enough for a meaningful computed trajectory. For an autonomous car, computing a trajectory for 1 second is ideal since the environment would change too much beyond that interval and wouldn't make much sense for further computation. Considering these factors, I found that N=10 and dt=0.1 provided better results than other values I tired.
 
Previous values tried for timestep-length/elapsed-duration include 5/0.2, 7/0.07, 16/0.08, 25/0.05, etc. 

### Polynomial Fitting and MPC Preprocessing
The car's waypoints are in the map's coordinate system. To simplify the process of fitting a polynomial through them, the waypoints are first tranformed to the vehicle's coordinate system. Doing allows sets the car's X,Y position to origin (0,0), and also the car's orientation to 0. These transformed waypoints are then fit to a 3rd degree polynomial. The polynomial is later used to compute the cross-track error (cte) and orientation error (epsi).
 
### Model Predictive Control with Latency
To account for latency (100 ms for this simulation), the state initial values need to be re-calibrated with respect to the delay interval. This is done by using the same kinematic model equations for the MPC, but with a timestep of 0.1 seconds (100 ms). These calibrated values are then used with the MPC instead of the state initial values.
