#include "PID.h"
#include <iostream> // debugging
using namespace	std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double i_deadband, double antiwindup_lim) {
	p_error=0.;
	i_error=0.;
	d_error=0.;

	this->Kp=Kp;
	this->Ki=Ki;
	this->Kd=Kd;

	this->i_deadband = .05;
	this->antiwindup_lim = 100;

}

void PID::UpdateError(double cte) {

	d_error= cte-p_error;
	p_error=cte;

	// getting clever here with deadband and reset at cte zero crossing
	if (cte>i_deadband) { // positive
		i_error_pos += cte-i_deadband;
		if (i_error_pos > antiwindup_lim) {
			i_error_pos = antiwindup_lim;
		}
		i_error_neg = 0.;
		i_error=i_error_pos;
	} else if (cte<-i_deadband) {
		i_error_neg += cte+i_deadband;
		if (i_error_pos < -antiwindup_lim) {
			i_error_pos = -antiwindup_lim;
		}
		i_error_pos = 0.;
		i_error=i_error_neg;
	}
	else {
		i_error_pos = 0.;
		i_error_neg = 0.;
	}

}

double PID::TotalError() {
	double Ppart, Ipart, Dpart;

	Ppart= -Kp*p_error;
	Ipart= -Ki*i_error;
	Dpart= -Kd*d_error;

	return (Ppart + Ipart + Dpart);
}

