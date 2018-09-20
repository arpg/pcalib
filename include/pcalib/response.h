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

    inline const std::string& Type() const
    {
      return type_;
    }

    inline int NumParams() const
    {
      return params_.size();
    }

    inline const Eigen::Vector2d& GetRange() const
    {
      return range_;
    }

    inline void SetRange(const Eigen::Vector2d& range)
    {
      PCALIB_ASSERT(range[0] < range[1]);
      range_ = range;
    }

    inline const Eigen::VectorXd& GetParams() const
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

    Eigen::Vector2d range_;

    Eigen::VectorXd params_;
};

} // namespace pcalib