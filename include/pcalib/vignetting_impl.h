#pragma once

#include <pcalib/vignetting.h>

namespace pcalib
{

template <typename Scalar, typename Derived>
class VignettingImpl : public Vignetting<Scalar>
{
  public:

    VignettingImpl(int width, int height) :
      Vignetting<Scalar>(width, height)
    {
      Initialize();
    }

    virtual ~VignettingImpl()
    {
    }

    Scalar operator()(Scalar u, Scalar v) const override
    {
      return Derived::GetVignetting(this->params_.data(), u, v,
          this->width_, this->height_);
    }

    virtual void Reset() override
    {
      Derived::ResetParameters(this->params_.data(),
          this->width_, this->height_);
    }

  private:

    void Initialize()
    {
      this->type_ = std::string(Derived::Type);
      this->params_.resize(Derived::GetNumParams(this->width_, this->height_));

      Derived::ResetParameters(this->params_.data(),
          this->width_, this->height_);
    }
};

} // namespace pcalib