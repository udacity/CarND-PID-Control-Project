# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
### Overview

The goal of this project is to implement a PID Controller to drive a car as fast as possibly on the track safely in the simulator. 

### Implementation
The PID controller is implemented in pid.cpp. The core function is as follows:
```cpp
 double PID::UpdateError(double cte) {
   d_error_m = (cte - p_error_m) / dt_m;
   i_error_m += cte * dt_m;
   p_error_m = cte;

   if (std::abs(cte) < deadband_m)
     return 0;

   double output;
   output = -(kp_m * p_error_m + ki_m * i_error_m + kd_m * d_error_m);
   output = output > max_output_m ? max_output_m : output;
   output = output < min_output_m ? min_output_m : output;

   return output;
 }
```
The output of the PID is composed of three terms, namely proportional term, integral term, and derivative term. The output of P term is linearly proportional to the cross track error. Larger error results in larger P output and thus faster recovery from deviation. However, large P output also makes the system unstable and oscillating. To solve this issue, we need to bring in the D term, which is propotional to the derivative of cross track error. That means that D term will always try to dampen the output. This is helpful because when the car is getting closer to the reference path, D term will slowly reduce the overall output and thus reduce the overshot. However, both terms failed to solve the static error issue, such as steering offset. To fix this issue, we need the I term, which uses the integral error. If there is system error, the error should accumulate over time and as it gets larger, the I term also has larger output and thus compensates the system error.

### Tuning
The way I tune the PID parameters is as follows
1. Set I and D coefficients to zero. And slowly increase the P coefficient. As the coefficient getting larger, I see the steering gets larger. I increase the P term util the point that the car is slightly oscillating. Then I move on to the next step.
1. To reduce the oscillation I slowly increase the D term. I could see the car's steering getting smoother and less overshot. Then increase the P term a little bit to make the recovery more quickly and then adjust the D term accordingly.
1. Lastly, I added the I term to solve the system error. After adding the I term, I also need to ajust the P and D term a little bit to find the best combination.

Apart from manual tuning, I also tried a automative tuning method. Essentially it measures the amplitude and period of the oscillation and then calculates the best parameters using (Ziegler–Nichols method)[https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method]. The challenge was to measure the correct amplitude and period in simulation. Due to the inflexibility of the simulator I didn't manage to measure these values easily and lastly I decided to go with manual tunning.
