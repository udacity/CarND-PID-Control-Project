#include "PID.h"
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID( double Kp, double Ki, double Kd, double cap) : Kp( Kp ), Ki( Ki ), Kd( Kd ), cap(cap)
{
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

PID::~PID() {}


void PID::UpdateError(double cte) {

    d_error = cte - p_error;
    i_error += cte;
    p_error = cte;   
}

double PID::TotalError() {
	return std::max( -cap, std::min( -Kp * p_error - Kd * d_error - Ki * i_error, cap ) );
}
