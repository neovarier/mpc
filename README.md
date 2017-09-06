# Model Predictive Control

## Model
A Kinematic vehicle model is used for modelling the motion of the car and controlling the path taken by the car
Kenematic model is a simplified dynamic model which uses the location (x,y), velocity(v) & orientation (psi) to model the vehicle motion. It also uses cross-track error(cte) and orientation-error(epsi) to track how much is it deviating from the desired trajectory in terms of distance & orientation. That gives the following state - [x,y,v,psi,cte,epsi]. The model has the steering angle(delta) and throttle(a) as the actuators that can control and update the state of the vehicle over time.

The following equations handle the update of state over a time interval of dt. They show the state of the vehicle at (t+dt)
- x(t+dt) = x(t) + v(t)*cos(psi(t))*dt
- y(t+dt) = x(t) + v(t)*sin(psi(t))*dt
- psi(t+dt) = psi(t) + (v(t)/Lf)*delta(t)*dt
- v(t+dt) = v(t) + a(t)*dt
- cte(t+dt) = cte(t) + v(t)*sin(psi(t))*dt
- epsi(t+dt) = epsi(t) + (v(t)/Lf)*delta(t)*dt

Lf measures the distance between the front of the vehicle and its center of gravity.
The model predicts the actuators to be applied (delta, a) so that the cross-track and orientation error minimizes.

## Timestep Length and Elapsed Duration (N & dt)

Over a few runs of the car on the track at an average of 40 mph, I considered a horizon time T of 1 second. 
I tried to tune N & dt keeping T=1 second (T=N*dt) with various combinations till I obtained the optimum set of values  (N,dt) which gave a good result. I tried (20,0.05), (15,0.65), (10,0.1), (5,0.2).
Based on the observation N=10 & dt=0.1 gave a good result.

## Polynomial Fitting and MPC Preprocessing

## Model Predictive Control with Latency
