# Model Predictive Control

The objective of this project was to implement a model predictive control in C++ to drive the car around the track. Further, the model parameters were tuned in order to reach maximal speed.

## Model
Our model is a kinematic model which is a simplification of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. 

### State Variables

The simulator provides state variables of the car and a series of waypoints with respect to the arbitrary global map coordinate system which we can use to fit a polynomial used to estimate the curve of the road ahead. 

| State			            | Description	        					                | 
|:---------------------:|:---------------------------------------------:| 
| px	                  | Current location of the vehicle in the x-axis of an arbitrary global map coordinate system.	| 
| py                    | Current location of the vehicle in the y-axis of an arbitrary global map coordinate system. |
| psi                   | Current orientation of the vehicle. |
| v                     | Current velocity of the vehicle. |

### Actuators

Actuator input allows to control the vehicle state. Most cars have actuators such as steering wheel, throttle and the brake pedal. For simplicity we will consider the throttle and brake pedals as a single actuator.

| Actuator			            | Description	        					                | 
|:---------------------:|:---------------------------------------------:| 
| delta | Steering angle of the vehicle which is between -25 and 25 degrees.
| a |  Throttle/Brake value with a range between [-1, 1], where negative values are signifying braking and positive values acceleration.

### Model Equations

The model can predict the state on the next time step by taking into account the current state and actuators as follows:

px(t+1) = px(t) + v(t) * cos(psi(t)) * dt <br>
py(t+1) = py(t) + v(t) * sin(psi(t)) * dt <br>
psi(t+1) = psi(t) + v(t) / Lf * delta * dt <br>
v(t+1) = v(t) + a(t) * dt <br>
cte(t+1) = cte(t) + v(t) * sin(epsi(t)) * dt <br>
epsi(t+1) = epsi(t) + v(t) / Lf * delta * dt <br>

where:

| Parameter			        | Description	            					            | 
|:---------------------:|:---------------------------------------------:| 
| Lf  | Distance between the front of the vehicle and its center of gravity|
| cte | Cross Track Error, which is the difference between our desired position and actual position. |
| epsi| Orientation Error, which is the difference between our desired orientation and actual orientation. |

## MPC 

Step 1) Define the length of the trajectory, N, and duration of each timestep, dt. I used N = 10 and dt = 100ms. Looking ahead too much is hurting the solution <br>
Step 2) Transform the points to vehicle's orientation. <br>
Step 3) Fit a 3rd-order polynomial to the x, y coordinates <br>
Step 4) Calculate cross track error and orientation error.



Actual state of the vehicle was "shifted" into the future by 100 ms latency. It helps to reduce negative effects of the latency and increase stability of the controller. The latency was introduced to simulate real delay of a human driver or physical actuators in case of a self driving car. 

The time horizon (T) was chosen to 2 s after experiments. It was shown that the MPC could drive safely around the track with T = 1 s, but on a slow speed. Higher speed requires more future information to make smart decisions in serial turns. Time step duration (dt) was setted equal to the latancy of the simulation (0.1 s), hense, 20 time steps (N) was used.

The cost function parameters were tuned by try-and-error method. All these parameters are stored in the src/MPC.h file. They were tuned in order to reach maximal speed and agressive race style with use of the whole width of the road and breaking before turns.

Step 1: Transform given coordinates from world to car coordinates
Step 2: Fit the points of the reference driving line provided using a third degree polynomial
Step 3: Choosing number of steps we will look ahead and time step. Choose N = 10 and dt = 100ms. Looking ahead too much is hurting the solution

