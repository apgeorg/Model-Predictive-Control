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

## Timestep Length and Elapsed Duration (N & dt)

The prediction horizon T is the duration over which future predictions are made.
T is the product of the number of timesteps N in the horizon and dt is the elapsed time between actuations. T should be as large as possible, while dt should be as small as possible.

A large N allows planning further ahead but it also impacts the controller performance. Small N values makes the car unstable cause of the small planning horizon.    

As mentioned above, the timestep duration dt needs to be as small as possible. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. 
While setting dt = 0.1s, it matches to the 100ms delay latency that we have for actuators, so it is easy to account for delay.
 
Finally, I choose N = 10 and dt = 0.1s. With this values it was shown that the car could drive safely and with a higher speed (~ 100mph) around the track.

## Polynomial Fitting and MPC Preprocessing

The provided waypoints by the simulator are transformed to vehicle coordinates. After that a 3rd order polynomial is fitted to the transformed waypoints. These polynomial coefficients are used to calculate the cte and epsi later on. They are used by the solver as well to create a reference trajectory.

## Model Predictive Control with Latency

The latency was introduced to simulate real delay of a human driver or physical actuators in case of a self driving car. 
To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. 
