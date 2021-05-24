/**
 * @file twiddle.cpp
 * 
 * @brief Class definition for Twiddle algorithm
 * 
 * @author M Talha
 */

#include "numeric" // std::accumulate
#include "twiddle.hpp"

Twiddle::Twiddle(const double (&pd)[3], const double threshold,
                 double distance_threshold, bool use_twiddle) : 
                 threshold_(threshold), distance_threshold_(distance_threshold),
                 use_twiddle_(use_twiddle)
{
  std::copy(pd, pd + 3, pd_);
}

void Twiddle::Tune(PID &pid)
{
  switch (delta_index_)
  {
  case 0:
    pid.Kp += pd_[delta_index_] * static_cast<int>(tuning_direction_);
    break;
  case 1:
    pid.Ki += pd_[delta_index_] * static_cast<int>(tuning_direction_);
    break;
  case 2:
    pid.Kd += pd_[delta_index_] * static_cast<int>(tuning_direction_);
    break;
  default:
    break;
  }
}

void Twiddle::UpdateBestParams()
{
  best_error_ = average_error_;
  best_distance_ = distance_count_;
  pd_[delta_index_] *= 1.1;
  tuning_direction_ = Direction::forward;
  CycleIndex();
}

void Twiddle::ResetParams(PID &pid)
{
  tuning_direction_ = Direction::forward;
  Tune(pid);
  pd_[delta_index_] *= 0.9;
  CycleIndex();
}

void Twiddle::CycleIndex()
{
  delta_index_ = (delta_index_ + 1) % 3;
}

void Twiddle::Reset()
{
  distance_count_ = 0;
  average_error_ = 0;
}

void Twiddle::LogStep(PID &pid)
{
  std::cout << "p: (" << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << "), ";
  std::cout << "pd: (" << pd_[0] << ", " << pd_[1] << ", " << pd_[2] << "), ";

  std::cout << "avg err: " << average_error_ << ", ";
  std::cout << "dist: " << distance_count_ << std::endl;
}

void Twiddle::LogState(PID &pid)
{
  std::cout << "Iteration: " << ++iterations_
            << ", best error: " << best_error_
            << ", best dist: " << best_distance_
            << ", current params: "
            << "Kp: " << pid.Kp
            << "Ki: " << pid.Ki
            << "Kd: " << pid.Kd
            << std::endl;
}