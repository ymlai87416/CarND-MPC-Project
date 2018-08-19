[//]: # (Image References)

[image1]: ./output/MPC.PNG "MPC equation"
[image2]: ./output/x_update.PNG "update x"
[image3]: ./output/y_update.PNG "update y"
[image4]: ./output/yaw_update.PNG "update yaw"
[image5]: ./output/v_update.PNG "update v"
[image6]: ./output/x.PNG "x"
[image7]: ./output/y.PNG "y"
[image8]: ./output/yaw.PNG "yaw"
[image9]: ./output/v.PNG "v"
[image10]: ./output/Lf.PNG "Lf"
[image11]: ./output/sigma.PNG "sigma"
[image12]: ./output/a.PNG "a"
[image13]: ./output/cte.PNG "cte"
[image14]: ./output/orientation_error.PNG "orientation error"
[image15]: ./output/cost_steering.PNG "cost steering"
[image16]: ./output/cost_high_change.PNG "cost high change"
[image17]: ./output/cost_distance.PNG "cost distance change"
[image18]: ./output/cost_reference_v.PNG "cost reference velocity"
[image19]: ./output/t_0.PNG "timestep 0"
[image20]: ./output/timestep_1_n.PNG "timestep 1 to n"
[image21]: ./output/final_cost_func.PNG "final cost function"
[image22]: ./output/optimization_prob.PNG "optimization"

# CarND-Controls-MPC
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

## Code Style

This project code style sticks to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Implementation

### Model
In this project, I build a kinematic model and use it with MPC to control the car via actuators.

The kinematic model only considers the car kinematic and does not consider other factors such as forces acting on the car,
slip angle, slip ration and the tire model etc. 

The kinematic model is a simple model hence it can be calculated in near real time, and the result is often accurate enough
to control the car under normal situation.

There is another kind of model called dynamic model which considers more factors and the model is more complex.

#### State
The primary state of the car are (![alt text][image6], ![alt text][image7], ![alt text][image8], ![alt text][image9]),
which represent the x-direction, y-direction, orientation and the velocity of the vehicle.

At time t, MPC applies (![alt text][image11], ![alt text][image12]) which is also considered as the state of the car. These states cannot be ignored when a delay is present.

Given a driving path, there are 2 additional states: cross track error and orientation error. These states represent how much 
the car derivate from the driving path.

#### Actuators
The control inputs / actuators are (![alt text][image11], ![alt text][image12]),
which represent the steering angle and the acceleration of the vehicle. 

#### Update equations
The update equations are as follow:

| Updated state          | Equation  |
|:-----:| :-----:|
| X-direction | ![alt text][image2] |
| Y-direction | ![alt text][image3] |
| Orientation | ![alt text][image4] |
| velocity | ![alt text][image5] |

where ![alt text][image10] measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower
the turn rate. When a vehicle is at higher speed, the car turns quicker than at lower speed, so v is also used to calculate the orientation of the car.


### Timestep length and elapse duration

In the case of driving a car, elapse duration should be a few seconds, at most. Beyond that horizon, the environment 
will change enough that it won't make sense to predict any further in the future.

For the timestep length, MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. 
Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous 
reference trajectory. This is sometimes called "discretization error".

To find out the timestep length and elapse duration, I try the following set of parameters.

Under reference velocity of 70mph, and delay of 0.1ms

| N | dt  | Result |
|:-----:| :-----| :-----|
| 10 | 0.2 | (Chosen) The car completed a lap with speed around 50mph |
| 10 | 0.1 | The car left the track |
| 10 | 0.15 | The car left the track |
| 10 | 0.5 | The car slowed down to 20-30mph and completed a lap |
| 20 | 0.2 | The car completed a lap with speed around 50mph |

After trying the above combination, I think that the combination  of N=10 and dt=0.2 is the best it is cheaper 
to calculate the result is smaller than N=20 and dt=0.2, and give the fastest speed without leaving the track.

When delay=0s, the car can complete a lap even when dt=0.1s, but where delay=0.1s, dt must be at least 0.2s 
to keep the car on the track in my implementation.

Increasing N beyond the threshold makes no difference because the car will only take the steering angle and acceleration values
of the first time interval [0, dt), which it is not likely to be affected by those future events.

Decreasing N below the threshold make the car more likely to crash at the curve because the car is not able to slow down
before making a turn.

### Polynomial fitting and MPC Preprocessing

Before using MPC to calculate the acceleration and the steering angle, a path is needed, and I use a
3rd order polynomial curve to present the desired path.

To compensate with the 0.1s delay, I have first predicted the state of the car after 0.1s, and use this as the origin of the
car coordination system. 

I then transform the path from world coordination system to the car coordination system.

Under the car coordination system, the initial state of the car is (x'=0, y'=0, psi'=0, v'=v+a*dt). I then calculate the
cross track error and the orientation error and pass them to the solver for the answer.

You can refer to this part of [main.cpp](https://github.com/ymlai87416/CarND-MPC-Project/blob/01f19a62560dd1954a607ecb501ce585f939d671/src/main.cpp#L112-L115) for actual implementation.

### Model Predictive Control with Latency

In this project, the Model Predictive Control has a pre-set latency of 100 milliseconds. The 100 millisecond is set to allow
MPC to calculate a more accurate solution and allow signal delay from other sensor components.

Because of the delay, I have discussed the necessary code changes I have made in the previous section. They are
1. Tuning of dt instead of using dt=0.1s
2. Predicting the car position and orientation after 0.1s and use it as the origin of the car coordination system.

#### The cost function
In order for the MPC to find the control input, besides the kinematic model, I have to provide a cost function so that MPC 
can compare which set of solution (![alt text][image11], ![alt text][image12]) is better.

Here are the possible cost function

| Errors / Soft constraint         | Description  | Equation |
|:-----:| :-----| :-----:|
| Cross track error | The error between the center of the road and the vehicle's position| ![alt text][image13] |
| Orientation error | The error penalizes if the car does not head the correct direction | ![alt text][image14] |
| Soft constraint: Destination | It penalized if the car is not at the destination | ![alt text][image17] |
| Soft constraint: Reference velocity | It penalized if the car is not driving at reference velocity | ![alt text][image18] |
| Soft constraint: Steering angle | It penalized if the steering angle is too much | ![alt text][image15] |
| Soft constraint: Rate of change in input | It penalized if the changes of the control inputs are too high | ![alt text][image16] |

#### Final cost function

In this project, the objective is to make the car drive around the track. 
Let define the current time as ![alt text][image19] and the MPC model predicts N time-steps forward, denoted these timestamps as
![alt text][image20]

Hence, the final cost function is 

![alt text][image21]

The optimization problem is:

![alt text][image22]


## Simulation

Here is the [result](https://www.youtube.com/watch?v=L0yz-75TPvI)
