#ifndef ONLINE_STATS_H
#define ONLINE_STATS_H
#include <vector>
#include <functional>

class OnlineStats
{
  // online standard deviation
  // see: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  // section on incremental computation
public:
  OnlineStats() : n_(0), K_(0.0), sum_(0.0), sum2_(0.0) {};
  virtual ~OnlineStats() {};

  virtual void addData(double x); // add data itme; only mathematical effect, the data is not stored
  void removeData(double x);      // remove data item; only mathematical effect, the data is not stored

  int n()      const { return n_; }
  double mean()   const { return K_ + sum_ / n_; }
  double varp()   const;      // variance of population
  double stdevp() const;      // standard deviation of population
  double stdevs() const;      // standard deviation of sample
  virtual void clear();       // reset
private:
  int n_;         // number of data items seen so far
  double K_;      // assumed mean - for numerical stability
  double sum_;    // sum of data items seen so far
  double sum2_;   // squared sum of data items seen so far
};

class MovingOnlineStats : public OnlineStats {
public:
  MovingOnlineStats(int window_size) : window_size_(window_size), i_(-1) { buffer_.reserve(window_size); }
  virtual ~MovingOnlineStats() {}
  void addData(double x) override;
  void clear() override;
private:
  int window_size_;           // number of moving items to track
  int i_;
  std::vector<double> buffer_;  // buffer
};
#endif /* ONLINE_STATS_H */
