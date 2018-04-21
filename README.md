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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Implementation

### Model
In this project, I build a kinematic model and use in with MPC to control the car via accelerator, break and steering wheel.

Kinematic model only considers the car kinematic and does not consider other factors such as forces acting on the car,
slip angle, slip ration and the tire model etc. 

The kinematic model is a simple model hence it can be calculated in near real time, and the result is often accurate enough
to control the car under normal situation.

There is another kind of model called dynamic model which considers more factors but it is more complex.

#### State
The vehicle state can be model in (![alt text][image6], ![alt text][image7], ![alt text][image8], ![alt text][image9]),
which represent the x direction, y direction, orientation and the velocity of the vehicle.

#### Actuators
The control inputs / actuators are (![alt text][image11], ![alt text][image12]),
which represent the turn rate and the acceleration of the vehicle. 

#### Update equations
The update equations and the constraint are as follow:

| Updated state          | Equation  |
|:-----:| :-----:|
| X direction | ![alt text][image2] |
| Y direction | ![alt text][image3] |
| Orientation | ![alt text][image4] |
| velocity | ![alt text][image5] |

where ![alt text][image10] measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower
the turn rate. When a vehicle is at higher speed, the vehicle turn quicker than at lower speed, so v is also used to calculate the orientation of the car.


### Timestep length and elapse duration

To find out the timestep length and elapse duration, I do a grid search on both the ?? and ??, and find that where ???=???
and ???=??? , MPC model gives the best result.

### Polynomial fitting and MPC Preprocessing
Before using MPC to calculate the acceleration and the steering angle for the current timestep, a path is needed, and I use a
3rd order polynomial curve to present the desired path.

### Model Predictive Control with Latency
In this project, the Model Predictive Control have a pre-set latency of 100 millisecond. The 100 millisecond is set to allow
MPC to calculate a more accurate solution and allow signal delay from other sensor components.


#### The cost function
In order for the MPC to find the control input, beside the kinematic model, I have to provide a cost function so that MPC 
can compare which solution (![alt text][image11], ![alt text][image12]) is better.

Here are the possible cost function

| Errors / Soft constraint         | Description  | Equation |
|:-----:| :-----| :-----:|
| Cross track error | The error between the center of the road and the vehicle's position as the cross track error | ![alt text][image13] |
| Orientation error | The error penalizes if the car does not head the correct direction | ![alt text][image14] |
| Soft constraint: Destination | It penalized if the car is not at the destination | ![alt text][image17] |
| Soft constraint: Reference velocity | It penalized if the car is not driving at reference velocity | ![alt text][image18] |
| Soft constraint: Steering angle | It penalized if the steering angle is to much | ![alt text][image15] |
| Soft constraint: Rate of change in input | It penalized if the changes of the control inputs is too high | ![alt text][image16] |

Final cost function

In this project, we would like the car to go at reference speed around the track. 
Let define the current time is ![alt text][image19] and the MPC model predicts N time-steps forward, denoted these timestamps as
![alt text][image20]

Hence, the final cost function is 

![alt text][image21]

The optimization problem is:

![alt text][image22]


## Simulation

Here is the result
