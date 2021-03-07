#include <cmath>
#include <limits>
#include "PID.h"


PID::PID(double Kp_, double Ki_, double Kd_) : Kp(Kp_), Ki(Ki_), Kd(Kd_)
{
    p_error = d_error = i_error = 0.0;
}

PID::~PID() = default;

void PID::UpdateError(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::Controller()
{
    double steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
    if (steer_value < -1)
    {
        steer_value = -1;
    }
    else if (steer_value > 1)
    {
        steer_value = 1;
    }
    return steer_value;
}
