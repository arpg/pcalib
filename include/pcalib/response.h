#pragma once

#include <Eigen/Eigen>
#include <pcalib/exception.h>
#include <pcalib/visitor.h>

namespace pcalib
{

template <typename Scalar>
class Response
{
  public:

    Response()
    {
    }

    virtual ~Response()
    {
    }

    const std::string& Type() const
    {
      return type_;
    }

    inline int NumParams() const
    {
      return params_.size();
    }

    inline const Eigen::VectorXd& GetParams() const
    {
      return params_;
    }

    inline Eigen::VectorXd& GetParams()
    {
      return params_;
    }

    inline void SetParams(const Eigen::VectorXd& params)
    {
      PCALIB_ASSERT(params.size() == params_.size());
      params_ = params;
    }

    virtual Scalar operator()(Scalar value) const = 0;

    virtual void Reset() = 0;

  protected:

    std::string type_;

    Eigen::VectorXd params_;
};

} // namespace pcalib