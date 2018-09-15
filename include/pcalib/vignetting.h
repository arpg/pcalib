#pragma once

#include<vector>
#include <pcalib/exception.h>
#include <pcalib/visitor.h>

namespace pcalib
{

class TrivialVignetting;

class VignettingVisitor
{
  public:

    virtual void Visit(const TrivialVignetting& vignetting) = 0;
};


class Vignetting
{
  public:

    virtual int parameter_count() const = 0;

    virtual std::vector<double> parameters() const = 0;

    virtual void set_parameters(const std::vector<double>& params) = 0;

    virtual void Accept(VignettingVisitor& visitor) const = 0;
};

class TrivialVignetting : public Vignetting
{
  public:

    int parameter_count() const override
    {
      return 0;
    }

    std::vector<double> parameters() const override
    {
      return std::vector<double>();
    }

    void set_parameters(const std::vector<double>& params) override
    {
      PCALIB_ASSERT_MSG(!params.empty(), "invalid parameter count");
    }

    void Accept(VignettingVisitor& visitor) const override
    {
      return visitor.Visit(*this);
    }

    inline double operator()(double x, double y) const
    {
      return 1.0;
    }

    template <typename Scalar>
    Scalar operator()(const Scalar* parameters, Scalar x, Scalar y) const
    {
      return Scalar(1);
    }
};

} // namespace pcalib