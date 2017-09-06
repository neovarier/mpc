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

To predict the optimized actuation values, the MPC solver is selected with a cost function which accumulates at every step till the horizon & this cost function is minimized.

The cost function is selected as:
* (cte(t+1)))^2 - To minimize the cross-track error
* (epsi(t+1))^2 - To minimize the orientation error
* (v(t+1) - 40)^2 - where 40 mph is the reference velocity. To minimize the deviation from velocity of 40 mph.
* (delta(t))^2 - To eliminate extreme actuations on steering wheel
* (a(t))^2 - To eliminate extreme actuations on throttle
* (delta(t+1) - delta(t))^2 - To minimize drastic changes in steering wheel actuation over subsequent times
* (a(t+1) - a(t))^2 - To minimize drastic changes in throttle actuation over subsequent times

## Timestep Length and Elapsed Duration (N & dt)

Over a few runs of the car on the track at an average of 40 mph, a horizon time T of 1 second was considered. 
I tried to tune N & dt keeping T=1 second (T=N*dt) with various combinations till I obtained the optimum set of values  (N,dt) which gave a good result. I tried (20,0.05), (15,0.65), (10,0.1), (5,0.2). Having larger N slowed down the prediction as it took more computation for predicting that many number of points. And large dt will gave coarse or incorrect predication.
Based on the observation N=10 & dt=0.1 gave good and fast result.

## Polynomial Fitting and MPC Preprocessing

 The reference waypoints and car position were transformed from Map coordinates to car coordinates. The car coordinates would be (0,0) and orientation would be 0 radians in car coordinates space. A 3rd degree polynomial is used to fit the reference waypoints.
 f(x) = a0 + a1*x + a2*x^2 + a3*x^3.
 The cross track error and orientation error are directly calculated as:
 cte = f(0)
 epsi = -arctan(f'(0))
 
Calculating the cte & epsi in Map coordinate space would involve a lot of mathematics. Instead it is easier to calculate them in car coordinate system. 
The calculated state [x,y,psi,cte,epsi] is fed to the MPC solver to predict the next steering angle and throttle to be applied.
It will also predict the new N predicted positions of the car.

## Model Predictive Control with Latency

The simulation system is set with a latency of 0.1 seconds. So a actuation of steering angle(delta) and throttle(a) would take into affect after 0.1 seconds. This can be taken into account by feeding MPC solver with the state of the vehicle that will occur after 0.1 seconds. This way the MPC solver will predict the steering angle & throttle that is to be applied after 0.1 seconds. Because of the system latency, these actuations would get applied only after 0.1 seconds which is when the vehicle would be at the desired state (state which is 0.1 seconds in the future). If this latency is not taken care, then the actuations will always have a delayed effect.

The the MPC quiz and the forum discussions were very helpful

## Result
A video of the output result is also uploaded - video.mp4
