#pragma once

#include <Eigen/Eigen>
#include <pcalib/exception.h>
#include <pcalib/visitor.h>

namespace pcalib
{

template <typename Scalar>
class Vignetting
{
  public:

    Vignetting(int width, int height) :
      width_(width),
      height_(height)
    {
    }

    virtual ~Vignetting()
    {
    }

    inline int Width() const
    {
      return width_;
    }

    inline int Height() const
    {
      return height_;
    }

    inline const std::string& Type() const
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

    virtual Scalar operator()(Scalar u, Scalar v) const = 0;

    virtual void Reset() = 0;

  protected:

    int width_;

    int height_;

    std::string type_;

    Eigen::VectorXd params_;
};

} // namespace pcalib