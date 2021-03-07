#include <cmath>
#include <limits>
#include "PID.h"


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = d_error = i_error = 0.0;
    dp = {0.1 * Kp, 0.1 * Kd, 0.1 * Ki};
    total_error = 0.0;
    best_error = std::numeric_limits<double>::max();
}

void PID::UpdateError(double cte)
{
    p_error = cte;
    d_error = cte - p_error;
    i_error += cte;
}

double PID::get_steering()
{
    double steer_value = -Kp * p_error - Kd * d_error - Ki * i_error;
    return steer_value;
}