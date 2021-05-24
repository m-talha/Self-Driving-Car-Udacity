/**
 * @file twiddle.hpp
 * 
 * @brief Class declaration for Twiddle algorithm
 * 
 * @author M Talha
 */

#pragma once
#include <iostream>
#include <iterator>
#include "PID.h"

/**
 * Perform parameter optimisation using Twiddle algorithm
 */
class Twiddle
{
public:
  Twiddle(const double (&pd)[3], const double threshold, 
      double distance_threshold = 3000, bool use_twiddle = true);

  Twiddle(const double threshold, double distance_threshold = 3000, 
      bool use_twiddle = true)
      : threshold_(threshold), distance_threshold_(distance_threshold),
        use_twiddle_(use_twiddle) {}

  void Run();
  // Modify PID parameters
  void Tune(PID &pid);
  // Get the sum of current delta values
  double SumPD() { return std::accumulate(pd_, (pd_ + 3), 0.0); }
  // Upon finding a new best error, update best error, increase delta
  // magnitude and reset tuning direction for next delta parameter
  void UpdateBestParams();
  // Upon failing to reduce best error when tuning values,
  // reset state for tuning with next delta
  void ResetParams(PID &pid);
  // Cycle to the next delta value for tuning
  void CycleIndex();
  // Log current step
  void LogStep(PID &pid);
  // Log current state
  void LogState(PID &pid);
  // Reached threshold distance
  bool DistanceReached() { return distance_count_ >= distance_threshold_; }
  void Reset();

  // Direction to modify pid parameters
  // 1 step forward, 2 steps back
  enum class Direction
  {
    forward = 1,
    backward = -2
  };
  Direction tuning_direction_ = Direction::forward;

  // Flag to enable twiddle parameter tuning
  bool use_twiddle_{};
  double threshold_{};
  // Current average error
  double average_error_{};
  // Current best error
  double best_error_{__DBL_MAX__};
  // Current best distance
  double best_distance_{0.};

  // Track distance covered
  double distance_count_{};
  // Current delta value to tune
  size_t delta_index_{};

private:
  double pd_[3]{1.0, 1.0, 1.0};
  size_t iterations_{};

  // Distance to tune parameters before aborting twiddle
  double distance_threshold_{};
};