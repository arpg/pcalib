#pragma once

#include <Eigen/Eigen>

namespace pcalib
{

class Vignetting
{
  public:

    virtual int parameter_count() const = 0;

    virtual Eigen::VectorXd parameters() const = 0;

    virtual void set_parameters(const Eigen::VectorXd& params) = 0;

    virtual double operator()(double x, double y) const = 0;
};

} // namespace pcalib