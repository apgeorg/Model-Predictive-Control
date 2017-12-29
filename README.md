# Model Predictive Control

The objective of this project was to implement a model predictive control in C++ to drive the car around the track. 
The cross track error (CTE) is calculated manually. In addition to that, there's a 100 millisecond latency between actuations commands on top of the connection latency. 

## Kinematic Models
Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. We want to derive a model that captures how the state evolves over time, and how we can provide an input to change it.

### State Variables

The simulator gives us the below state variables of the car and a series of waypoints with respect to the arbitrary global map coordinate system which we can use to fit a polynomial used to estimate the curve of the road ahead. 


| State			            | Description	        					                | 
|:---------------------:|:---------------------------------------------:| 
| px	                  | Current location of the vehicle in the x-axis of an arbitrary global map coordinate system.	| 
| py                    | Current location of the vehicle in the y-axis of an arbitrary global map coordinate system. |
| psi                   | Current orientation of the vehicle. |
| v                     | Current velocity of the vehicle. |
