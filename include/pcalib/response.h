#pragma once

#include <Eigen/Eigen>

namespace pcalib
{

class Response
{
  public:

    virtual int parameter_count() const = 0;

    virtual Eigen::VectorXd parameters() const = 0;

    virtual void set_parameters(const Eigen::VectorXd& params) = 0;
};

} // namespace pcalib