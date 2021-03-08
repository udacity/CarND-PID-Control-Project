Project: PID Controller
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Introduction

*In this project we'll revisit the lake race track from the [Behavioral Cloning Project](https://github.com/snandasena/behavioral-cloning). This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track!*

*The simulator will be provided the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.*

### Discussions

#### PID Controller
* P - Proportional
* I - Intergral
* D - Differential

|<img src="data/pid-img.png" width="500" height="250" />| <img src="data/pid.png" width="500" height="250" /> |
|-------------------------------------------------------------|-----------------------------------------------|
| Source: https://thinkautonomous.medium.com/ | Source: Udacity|

#### Steering PID Controller
The steering PID controller analyzes the cross track error(CTE) provided by the simulator and calculates the streering angle that tries to bring the car back to the center of the lane line. 

###### P - Proportional
The **P** is the most directly observable effect on the car's behaviour.It causes the car to steer propartionally to the car's distance from the lane center. Bigger values of P result in faster reactions of the steering angle with respect to the CTE.

###### D - Differential
The **D** counteracts the P component's tendency to overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly. Higher values of P also required higher values of D.

###### I - Integral
The **I** counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift.

#### Throttle PID Controller
Throttle PID controller was used to control the speed of the car via the throttle. The target speed is reduced if there in case of big CTE and high steering angle.


###### P - Proportional
The **P** is the most directly observable effect on the car's behaviour. High values of P resulted in quick reactions on changes of the target speed which even allowed the car to brake.

###### D - Differential
The **D** counteracts the P component's tendency to overshoot the target speed.

###### I - Integral
The **I** was set to a small value(0.00009) since the target speed is constantly changing and I observed a tendency to not breaking fast enough
