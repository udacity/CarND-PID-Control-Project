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

  //get the last line of the csv file
  this->parameter_file.exceptions(ifstream::failbit | ifstream::badbit);

  try{
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
    this->parameter_file.close();

    this->p[0]  = parameter_vector[this->KP];
    this->p[1]  = parameter_vector[this->KD];
    this->p[2]  = parameter_vector[this->KI];
    this->dp[0] = parameter_vector[this->DKP];
    this->dp[1] = parameter_vector[this->DKD];
    this->dp[2] = parameter_vector[this->DKI];
    this->index_for_twiddle = parameter_vector[this->TWIDDLE_INDEX];
    this->which_scope_in_twiddle = parameter_vector[this->WHICH_SCOPE];
    this->best_err = parameter_vector[this->BEST_ERROR];
  }catch (const fstream::failure& error) {
    cout << "Fail to open the csv file. You need to create " << file_name << "in ./build." << endl;
  }
  return parameter_vector;
}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  this->timer_for_command.reset();
  this->timer_for_episode.reset();

  this->total_error = 0.0;
  this->total_distance = 0.0;

  this->initial_command = true;
}

void PID::UpdateError(double cte) {
  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;
}

double PID::ComputeSteer(){
  double steer = (-this->Kp*this->p_error)  - (this->Kd * this->d_error) - (this->Ki * this->i_error);
  if (steer < -1.0){
    steer = -1.0;
  }else if(steer > 1.0){
    steer = 1.0;
  }
  return steer;
}

double PID::ComputeDeltaTime(){
//  clock_t end_time = clock();
//  double elapsed_time_seconds = double(end_time - this->begin_time)/CLOCKS_PER_SEC;
//  this->begin_time = end_time;

  double end_time2 = this->timer_for_command.elapsed();
  this->timer_for_command.reset();
  return end_time2;
}

double PID::ComputeTotalDistance(double delta_distance){
  this->total_distance += delta_distance;
  return this->total_distance;
}

double PID::TotalError() {
  this->total_error += fabs(this->p_error);
  return this->total_error;
}

double PID::Twiddle(double tol, double err){

  //Twiddle algorithm
  int ti = this->index_for_twiddle;
  if(this->which_scope_in_twiddle==0){
    this->p[ti] += this->dp[ti];
  }
  if((err < this->best_err) && (this->which_scope_in_twiddle==0)){
    this->best_err = err;
    this->dp[ti] *= 1.1;
    this->index_for_twiddle = (this->index_for_twiddle+1)%3;
  }else if((err >= this->best_err) && (this->which_scope_in_twiddle==0)){
    this->p[ti] -= 2.0*this->dp[ti];
    this->which_scope_in_twiddle=1;

    std::cout << this->p[0] << "," << this->p[1] << "," << this->p[2] << "," << this->dp[0] << "," <<  this->dp[1] <<"," <<  this->dp[2] << std::endl;

    return this->best_err ;
  }

  if(this->which_scope_in_twiddle==1){
    if(err < this->best_err){
      this->best_err = err;
      this->dp[ti] *= 1.1;
    }else{
      this->p[ti] += this->dp[ti];
      this->dp[ti] *= 0.9;
    }
    this->index_for_twiddle = (this->index_for_twiddle+1)%3;
    this->which_scope_in_twiddle = 0;
  }

  std::cout << this->p[0] << "," << this->p[1] << "," << this->p[2] << "," << this->dp[0] << "," <<  this->dp[1] <<"," <<  this->dp[2] << std::endl;

  return this->best_err ;
}

void PID::LogData(string file_name, std::vector<float> parameter_vector){
  fstream log_file;
  log_file.open(file_name, fstream::app);

  if (log_file.is_open())
  {
    cout << "log file is open" <<endl;
    log_file << parameter_vector[0]+1 << ",";
    log_file << this->p[0] << ",";
    log_file << this->p[1] << ",";
    log_file << this->p[2] << ",";
    log_file << this->dp[0] << ",";
    log_file << this->dp[1] << ",";
    log_file << this->dp[2] << ",";
    log_file << this->index_for_twiddle << ",";
    log_file << this->which_scope_in_twiddle << ",";
    log_file << this->best_err << ",";
    log_file << this->total_error << ",";
    log_file << this->total_distance << ",";
    log_file << 70.0 << "," << std::endl;
    log_file.close();
  }
  std::cout << std::endl;
  std::cout <<"final result is " << this->total_error - 1000.0*this->total_distance << std::endl;
}
