# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

OVERVIEW:-

The P, I, and D stand for Proportional, Derivative, and Integral gain. A PID controller is used here to calculate the steering angle of the simulated car, when the distance from the centre of the road is known. The objective of the project is to continously update steering angles such that the car always stays in the center of the road. The PID controller calculates an error value (cross track error) between the desired trajectory and the current position and applies a correction to a parameter(in this case steering angle) based on proportional, integral and differential terms.
PID controller will help us keep the CTE (cross-track error) as small as possible.
The first term (Kp) is proportional to the cross track error (cte) between the required target value and the actual value. The second term (Kd) is proportional to the derivative of the error and the last term (Ki) is proportional to the integral of the error.If Kp is too large, this can cause the system to start to oscillate due to too much input being applied. When Kp is small, the vehicle will slowly turn in the correct direction to reduce the error. Kd controls the rate of change of the error. Because of this, it can be used to either dampen or exaggerate the effects of the proportional term. An appropriate value can be used to smooth the output.Ki is used to counter the steady-state error or the drift in built in the vehicle. If there is no apparent drift then this component would keep increasing and would cause larger changes to the correction.

The values of the P, I, and D hyperparameters were fine tuned manually to:
P =-0.125; I = 0 D = -2.5
The P coefficient was chosen to minimize the oscillations from one edge to the other. The D coefficient was chosen so as the system recovers smoothly. The I coefficient was chosen to be 0 as the car running in the simulator did not seem to have any drift.

