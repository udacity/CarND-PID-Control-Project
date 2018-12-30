#pragma once
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include <unistd.h>
#include <math.h>
#include <cmath>


using namespace std;

double Kp_default = 0.1;
double Kd_default = 0;
double Ki_default = 0;


	// char buffer[256];
	// char *val = getcwd(buffer, sizeof(buffer));
	// if (val) {
	// 	std::cout << buffer << std::endl;
	// }


// bool file_exists(const std::string& filename)
// {
//     std::ifstream f(filename.c_str());
//     if (f.good() == 0) {
// 		return true;
// 	}
// 	else {
// 		return false;
// 	}
// }

class twiddle
{
public:
	double Kp, Kd, Ki;
	double dKp, dKd, dKi;
	vector<double> K, dK;
	twiddle();

	~twiddle() {}
	std::vector<double> get_params();
	std::vector<double> set_params(const double Kp, const double Kd, const double Ki, const unsigned int& tune_param);
	void update(const double& cte);
	void online_update(const double& cte);

private:
	const std::string parameters_file = std::string("../parameters.csv");
	unsigned int tune_param;
	unsigned int counter;
	unsigned int total_error;
	unsigned int iteration, twiddle_stage;
	unsigned int limit;
	double best_error;
	bool updated = false;
};


twiddle::twiddle()
{
	this->Kp = Kp_default;
	this->Kd = Kd_default;
	this->Ki = Ki_default;
	this->tune_param = 0;
	this->counter = 0;
	this->total_error = 0;

	std::string line;
	std::ifstream params_stream(this->parameters_file.c_str());
	if (!params_stream.is_open()) {
		throw "parameters file wont open";
	}

	/* parameters file
		0 			: iteration
		0			: twiddle_stage
		0.1			: Kp
		0			: Ki
		0			: Kd
		1			: dKp
		1			: dKd
		1			: dKi
		1			: tune param
		10000	 	: best error
		1000		: limit
	*/
	params_stream >> this->iteration;
	params_stream >> this->twiddle_stage;
	params_stream >> this->Kp;
	params_stream >> this->Ki;
	params_stream >> this->Kd;

	params_stream >> this->dKp;
	params_stream >> this->dKi;
	params_stream >> this->dKd;

	params_stream >> this->tune_param;
	params_stream >> this->best_error;
	params_stream >> this->limit;

	params_stream.close();

	cout << "iteration " << this->iteration << endl;
	cout << "tune param " << this->tune_param << endl;
	cout << "twiddle stage " << this->twiddle_stage << endl;

	// perform twiddle update
	if (0 == this->tune_param && 0 == this->twiddle_stage) {
		this->Kp += this->dKp;
	}
	else if (1 == this->tune_param && 0 == this->twiddle_stage) {
		this->Ki += this->dKi;
	}
	else if (2 == this->tune_param && 0 == this->twiddle_stage) {
		this->Kd += this->dKd;
	}
}

std::vector<double> twiddle::get_params()
{
	return std::vector<double>({this->Kp, this->Kd, this->Ki});
}

std::vector<double> twiddle::set_params(const double Kp, const double Kd, const double Ki, const unsigned int& tune_param)
{
	std::ofstream params_stream(this->parameters_file.c_str());
	if (!params_stream.is_open()) {
		throw "Division by zero condition!";
	}
}

void twiddle::online_update(const double& cte)
{
	this->counter++;
	cout << this->counter << endl;
	if (this->counter < limit) {
		return;
	}

	double max_tol = 0.2;

	std::vector<double> P({this->Kp, this->Kd, this->Ki});
	std::vector<double> dP({this->dKp, this->dKd, this->dKi});
    for (int i = 0; i < P.size(); i++)
	{
		if (i != this->tune_param) {
			continue;
		}

		if (this->twiddle_stage == 0)
		{
			if (abs(cte) < this->best_error)
			{
				this->best_error = abs(cte);
				dP[i] *= 1.1;
				// done, next parameter
				this->tune_param += 1;
				this->tune_param %= P.size();
				this->iteration += 1;
			}
			else // proceed to next run, same parameter
			{
				P[i] -= 2 * dP[i];
				this->twiddle_stage += 1;
			}
		}
		else if (this->twiddle_stage == 1)
		{
			if (abs(cte) < this->best_error)
			{
				this->best_error = abs(cte);
				dP[i] *= 1.1;
			}
			else {
				P[i] += dP[i];				
				dP[i] *= 0.9;
			}
			this->twiddle_stage = 0;
			this->tune_param += 1;
			this->tune_param %= P.size();
			this->iteration += 1;
		}
		else {
			throw "twiddle stage is neigher 0/1. what the ..";
		}
	}

}



void twiddle::update(const double& cte)
{
	this->counter++;
	this->total_error += abs(cte);
	cout << this->counter << endl;
	if (this->counter < limit || this->updated) {
		return;
	}

	double err = this->total_error / this->counter;

	std::vector<double> P({this->Kp, this->Kd, this->Ki});
	std::vector<double> dP({this->dKp, this->dKd, this->dKi});
	bool finished = false;
	int i = 0;

	cout << endl << endl << "----------------------------------------------" << endl;
    while (!finished)
	//for (int i = 0; i < P.size(); i++)
	{
		if (i != this->tune_param) {
			i++;
			continue;
		}

		if (this->twiddle_stage == 0)
		{
			if (err < this->best_error)
			{
				cout << i << " : stage 0 : abs(err) < this->best_error" << endl;

				this->best_error = abs(cte);
				dP[i] *= 1.1;
				// done, next parameter
				this->tune_param += 1;
				this->tune_param %= P.size();
				this->iteration += 1;
			}
			else // proceed to next run, same parameter
			{
				cout << i << " : stage 0 : abs(err) > this->best_error -> stage 1" << endl;
				P[i] -= 2 * dP[i];
				this->twiddle_stage += 1;
			}
		}
		else if (this->twiddle_stage == 1)
		{
			if (err < this->best_error)
			{
				cout << i << " : stage 1 : abs(cte) < this->best_error" << endl;
				this->best_error = err;
				dP[i] *= 1.1;
			}
			else {
				cout << i << " : stage 1 : abs(cte) > this->best_error" << endl;
				P[i] += dP[i];				
				dP[i] *= 0.9;
			}
			this->twiddle_stage = 0;
			this->tune_param += 1;
			this->tune_param %= P.size();
			this->iteration += 1;
		}
		else {
			throw "twiddle stage is neigher 0/1. what the ..";
		}
		finished = true;
		i++;
	}
	cout << endl << endl << "----------------------------------------------" << endl;

	// dump results
	std::ofstream params_stream(this->parameters_file.c_str());
	if (!params_stream.is_open()) {
		throw "parameters file wont open";
	}

	/* parameters file
		0 			: iteration
		0			: twiddle_stage
		0.1			: Kp
		0			: Ki
		0			: Kd
		1			: dKp
		1			: dKd
		1			: dKi
		1			: tune param
		10000	 	: best error
	*/
	params_stream << this->iteration << std::endl;
	params_stream << this->twiddle_stage << std::endl;
	params_stream << this->Kp << std::endl;
	params_stream << this->Ki << std::endl;
	params_stream << this->Kd << std::endl;

	params_stream << this->dKp << std::endl;
	params_stream << this->dKi << std::endl;
	params_stream << this->dKd << std::endl;

	params_stream << this->tune_param << std::endl;
	params_stream << this->best_error << std::endl;
	params_stream << this->limit << std::endl;	
	params_stream.close();

	this->updated = true;
	cout << "updated" << endl;
}


