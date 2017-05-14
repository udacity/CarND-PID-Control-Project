#include "OnlineStats.h"
#include <cmath>

void OnlineStats::addData(double x) {
  if (n_ == 0) K_ = x;
  auto d = x - K_;
  sum_ += d;
  sum2_ += d*d;
  ++n_;
}

void OnlineStats::removeData(double x) {
  auto d = x - K_;
  sum_ -= d;
  sum2_ -= d*d;
  --n_;
}

double OnlineStats::stdevp() const {
  //if (n_ < 1) return 0.0;
  return sqrt(varp());
}

double OnlineStats::varp() const {
	if (n_ < 1) return 0.0;
	return ((sum2_ - (sum_*sum_) / n_) / n_);
}

double OnlineStats::stdevs() const {
  if (n_ < 1) return 0.0;
  return sqrt((sum2_ - (sum_*sum_) / n_) / (n_-1));
}

void OnlineStats::clear() {
  n_ = 0; K_ = sum_ = sum2_ = 0.0;
}

/* virtual */ void MovingOnlineStats::addData(double x) {
  i_++;
  if (i_ == window_size_) i_ = 0;

  if (buffer_.size() < window_size_) { buffer_.push_back(x); }
  else { removeData(buffer_[i_]); buffer_[i_] = x; }
  OnlineStats::addData(x);
}

/* virtual */ void MovingOnlineStats::clear() {
  i_ = -1; buffer_.clear(); buffer_.reserve(window_size_);
  OnlineStats::clear();
}