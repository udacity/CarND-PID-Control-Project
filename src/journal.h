#ifndef __JOURNAL_H
#define __JOURNAL_H

#include <fstream>
#include <string>


class Journal {
public:
  Journal() {

  }

  virtual ~Journal() {
  }

  virtual void WriteHeader() {

  }

  virtual void Write(double cte, double steering_angle, double throttle) {

  }

  virtual void Close() {

  }
};



class JournalFile : public Journal {
public:
  JournalFile(const std::string &file_name)
    : ofs_(file_name, std::ofstream::out) {
  }

  ~JournalFile() {
  }

  void WriteHeader() {
    ofs_ << "cte;steering_angle;throttle;" << std::endl;
  }

  void Write(double cte, double steering_angle, double throttle) {
    ofs_ << cte << ";" << steering_angle << ";" << throttle << ";" << std::endl;
  }

private:
  std::ofstream ofs_;
};

#endif
