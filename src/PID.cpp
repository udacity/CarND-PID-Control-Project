#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
	this->p_error = 0;
	this->i_error = 0;
	this->d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

}

void PID::UpdateError(double cte) {
	this->d_error = cte - this->p_error;
	this->i_error += cte;
	this->p_error = cte;
}

double PID::TotalError() {
	double steering = - this->Kp * this->p_error - this->Ki * this->i_error - this->Kd * this->d_error;
	double p_ratio = 1;
	double i_ratio = this->Ki * this->i_error / (this->Kp * this->p_error);
	double d_ratio = this->Kd * this->d_error / (this->Kp * this->p_error);
	
	std::cout << "Proportional : " << std::setprecision (10) << std::setw(5) << this->Kp << ", " << this->p_error << ", " <<  -this->Kp * this->p_error << ", " << p_ratio << std::endl;
	std::cout << "Integral     : " << std::setprecision (10) << std::setw(5) << this->Ki << ", " << this->i_error << ", " << -this->Ki * this->i_error << ", " << i_ratio <<  std::endl;
	std::cout << "Derivative   : " << std::setprecision (10) << std::setw(5) << this->Kd << ", " << this->d_error << ", " << -this->Kd * this->d_error << ", " << d_ratio <<  std::endl;
	std::cout << "Total        : " << steering << std::endl;

	steering = (steering > 1 ? 1 : steering);
	steering = (steering < -1 ? -1 : steering);

	// if (this->p_error > 0 && steering > 0) {
	// 	steering /= 2;
	// }
	// else if (this->p_error < 0 && steering < 0) {
	// 	steering /= 2;
	// }
	return steering;
}

