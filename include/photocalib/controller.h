#pragma once

#include <cmath>
#include <limits>
#include <photocalib/exception.h>

#include <iostream>

namespace photocalib
{

class Controller
{
  protected:

    static constexpr double epsilon = 1E-8;

  public:

    Controller() :
      setpoint_(0.0),
      proportional_gain_(0.0),
      integral_gain_(0.0),
      integral_(0.0),
      lower_bound_(-std::numeric_limits<double>::max()),
      upper_bound_(+std::numeric_limits<double>::max()),
      integral_delay_(0),
      iteration_(0)
    {
    }

    inline int iteration() const
    {
      return iteration_;
    }

    inline double integral() const
    {
      return integral_;
    }

    inline double setpoint() const
    {
      return setpoint_;
    }

    inline void set_setpoint(double setpoint)
    {
      setpoint_ = setpoint;
      Reset();
    }

    inline double proportional_gain() const
    {
      return proportional_gain_;
    }

    inline void set_proportional_gain(double gain)
    {
      proportional_gain_ = gain;
    }

    inline double integral_gain() const
    {
      return integral_gain_;
    }

    inline void set_integral_gain(double gain)
    {
      integral_gain_ = gain;
    }

    inline int integral_delay() const
    {
      return integral_delay_;
    }

    inline void set_integral_delay(int delay)
    {
      PHOTOCALIB_ASSERT_MSG(delay >= 0, "delay cannot be negative");
      integral_delay_ = delay;
    }

    inline double lower_bound() const
    {
      return lower_bound_;
    }

    inline void set_lower_bound(double bound)
    {
      lower_bound_ = bound;
    }

    inline double upper_bound() const
    {
      return upper_bound_;
    }

    inline void set_upper_bound(double bound)
    {
      upper_bound_ = bound;
    }

    inline double Update(double feedback, double state)
    {
      ++iteration_;

      double bound;
      const double error = setpoint_ - feedback;
      const bool constrained = Constrained(error, state, bound);
      return constrained ? bound : Clamp(state + Update(error));
    }

    inline void Reset()
    {
      integral_ = 0.0;
      iteration_ = 0;
    }

  protected:

    inline double Clamp(double update) const
    {
      return std::min(upper_bound_, std::max(lower_bound_, update));
    }

    inline bool Constrained(double error, double state, double& bound) const
    {
      bound = (error > 0) ? upper_bound_ : lower_bound_;
      return LowerConstrained(error, state) || UpperConstrained(error, state);
    }

    inline bool LowerConstrained(double error, double state) const
    {
      return (error < 0 && state < lower_bound_ + epsilon);
    }

    inline bool UpperConstrained(double error, double state) const
    {
      return (error > 0 && state > upper_bound_ - epsilon);
    }

    inline double Update(double error)
    {
      return ProportionalUpdate(error) + IntegralUpdate(error);
    }

    inline double ProportionalUpdate(double error)
    {
      return proportional_gain_ * error;
    }

    inline double IntegralUpdate(double error)
    {
      return CanUpdateIntegral() ? ActualIntegralUpdate(error) : 0.0;
    }

    inline bool CanUpdateIntegral() const
    {
      return iteration_ > integral_delay_;
    }

    inline double ActualIntegralUpdate(double error)
    {
      integral_ += error;
      return integral_gain_ * integral_;
    }

  protected:

    double setpoint_;

    double proportional_gain_;

    double integral_gain_;

    double integral_;

    double lower_bound_;

    double upper_bound_;

    int integral_delay_;

    int iteration_;
};

} // namespace photocalib