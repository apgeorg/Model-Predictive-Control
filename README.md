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
| cte | Difference between our desired position and actual position. |
| epsi| Difference between our desired orientation and actual orientation. |

## MPC algorithm

Step 1) Define the length of the trajectory, N, and duration of each timestep, dt.
Step 2) Define vehicle dynamics and actuator limitations along with other constraints.
Step 3) Define the cost function.


