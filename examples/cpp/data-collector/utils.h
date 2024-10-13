//
// Created by keunjun on 24. 10. 13.
//

#ifndef RBY1_SDK_UTILS_H
#define RBY1_SDK_UTILS_H

#include <limits>

class Statistics {
 public:
  Statistics() { reset(); }

  void reset() {
    count_ = 0;
    sum_ = 0.;
    min_ = std::numeric_limits<double>::max();
    max_ = -std::numeric_limits<double>::max();
  }

  void add(double value) {
    count_++;
    sum_ += value;
    if (min_ > value) {
      min_ = value;
    }
    if (max_ < value) {
      max_ = value;
    }
  }

  int count() {
    return count_;
  }

  double avg() {
    if (count_ == 0) {
      return 0.;
    }
    return sum_ / count_;
  }

  double min() { return min_; }

  double max() { return max_; }

 private:
  int count_;
  double sum_;
  double min_;
  double max_;
};

#endif  //RBY1_SDK_UTILS_H
