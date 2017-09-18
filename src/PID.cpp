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
//  for(int i =0; i< parameter_vector.size(); i++){
//    cout << parameter_vector[i] << endl;
//  }
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
  this->begin_time_duration = clock();
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

  double end_time2 = this->timer_for_command.elapsed();
  this->timer_for_command.reset();

//  double diff = end_time2 - start_time;
//  this->start_time = end_time2;

  std::cout << "time=" << end_time2 << std::endl;

  return end_time2;
//  return elapsed_time_seconds;
}

double PID::ComputeTotalDistance(double delta_distance){
  this->total_distance += delta_distance;
}

double PID::TotalError() {
  this->total_error += fabs(this->p_error);
}

double PID::Twiddle(double tol, double err){

  //Twiddle algorithm
  int ti = this->index_for_twiddle;

  this->p[ti] += this->dp[ti];
  if((err < this->best_err) && (this->which_scope_in_twiddle==0)){
    this->best_err = err;
    this->p[ti] *= 1.1;
    this->index_for_twiddle = (this->index_for_twiddle+1)%3;
  }else if((err >= this->best_err) && (this->which_scope_in_twiddle==0)){
    this->p[ti] -= 2*this->dp[ti];
    this->which_scope_in_twiddle=1;
    return this->best_err ;
  }

  if(this->which_scope_in_twiddle==1){
    if(err < this->best_err){
      this->best_err = err;
      this->p[ti] *= 1.1;
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

}
//Make this tolerance bigger if you are timing out!
//def twiddle(tol=0.2):
//    # Don't forget to call `make_robot` before every call of `run`!
//    p = [0, 0, 0]
//    dp = [1, 1, 1]
//    robot = make_robot()
//    x_trajectory, y_trajectory, best_err = run(robot, p)
//
//    it = 0
//    while sum(dp) > tol:
//        print("Iteration {}, best error = {}".format(it, best_err))
//        for i in range(len(p)):
//            p[i] += dp[i]
//            robot = make_robot()
//            x_trajectory, y_trajectory, err = run(robot, p)
//
//            if err < best_err:
//                best_err = err
//                dp[i] *= 1.1
//            else:
//                p[i] -= 2 * dp[i]
//                robot = make_robot()
//                x_trajectory, y_trajectory, err = run(robot, p)
//
//                if err < best_err:
//                    best_err = err
//                    dp[i] *= 1.1
//                else:
//                    p[i] += dp[i]
//                    dp[i] *= 0.9
//        it += 1
//
//    # TODO: twiddle loop here
//
//    return p, best_err
//
//
