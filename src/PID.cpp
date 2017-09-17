#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

std::vector<float> PID::GetParamters(string file_name){
  string line;
  string lastline;
  std::vector<float> parameter_vector;


  //get the last line
  this->parameter_file.open(file_name);
  while (getline (this->parameter_file,line))
  {
     lastline=line;
  }

  //split the last line into parameter_vector
  std::istringstream iss(lastline);
  std::string token;

  while(std::getline(iss, token, ',')) {
      parameter_vector.push_back(stof(token));
  }
  for(int i =0; i< parameter_vector.size(); i++){
    cout << parameter_vector[i] << endl;
  }
  this->parameter_file.close();
  return parameter_vector;

}
void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  this->begin_time = clock();
  this->total_error = 0.0;
  this->total_distance = 0.0;
}

void PID::UpdateError(double cte) {
  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;
//  steer = -tau_p*cte  - tau_d * v - tau_i * int_cte
//  cout<< "CTE=" << cte << "," <<  p_error << "," << d_error << "," << i_error << std::endl;
}

double PID::ComputeSteer(){
  double steer = (-this->Kp*this->p_error)  - (this->Kd * this->d_error) - (this->Ki * this->i_error);
//  cout << "original steer = " << steer << endl;
  if (steer < -1.0){
    steer = -1.0;
  }else if(steer > 1.0){
    steer = 1.0;
  }
  return steer;
}

double PID::ComputeDeltaTime(){
  clock_t end_time = clock();
  double elapsed_time_seconds = double(end_time - this->begin_time)/CLOCKS_PER_SEC;
  this->begin_time = end_time;
  return elapsed_time_seconds;
}

double PID::ComputeTotalDistance(double delta_distance){
  this->total_distance += delta_distance;
}

double PID::TotalError() {
  this->total_error += fabs(this->p_error);
}
