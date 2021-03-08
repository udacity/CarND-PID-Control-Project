Project: PID Controller
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Introduction

*In this project we'll revisit the lake race track from the [Behavioral Cloning Project](https://github.com/snandasena/behavioral-cloning). This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track!*

*The simulator will be provided the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.*

### Discussions

The streering PID controller analyzes the cross track error(CTE) provided by the simulator and calculates the streering angle that tries to bring the car back to the center of the lane line. 


#### PID Controller
* P - Proportional
* I - Intergral
* D - Differential

|<img src="data/pid-img.png" width="600" height="250" />|
|-------------------------------------------------------------|


###### P - Proportional

The **P** is the most directly observable effect on the car's behaviour. 
