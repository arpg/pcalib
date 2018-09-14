#pragma once

#include <Eigen/Eigen>
#include <pcalib/exception.h>
#include <pcalib/visitor.h>

namespace pcalib
{

class LinearResponse;
class PolynomialResponse;

class ResponseVisitor
{
  public:

    virtual void Visit(const LinearResponse& response) = 0;

    virtual void Visit(const PolynomialResponse& response) = 0;
};

class Response
{
  public:

    virtual int parameter_count() const = 0;

    virtual Eigen::VectorXd parameters() const = 0;

    virtual void set_parameters(const Eigen::VectorXd& params) = 0;

    virtual void Accept(ResponseVisitor& visitor) const = 0;

    virtual void Reset() = 0;
};

class LinearResponse : public Response
{
  public:

    LinearResponse()
    {
    }

    int parameter_count() const override
    {
      return 0;
    }

    Eigen::VectorXd parameters() const override
    {
      return Eigen::VectorXd();
    }

    void set_parameters(const Eigen::VectorXd& params) override
    {
      PCALIB_ASSERT_MSG(params.size() > 0, "invalid parameter count");
    }

    inline double operator()(double value) const
    {
      return value;
    }

    template <typename Scalar>
    Scalar operator()(const Scalar* parameters, Scalar value) const
    {
      return value;
    }

    void Accept(ResponseVisitor& visitor) const override
    {
      visitor.Visit(*this);
    }

    void Reset() override
    {
    }
};

} // namespace pcalib