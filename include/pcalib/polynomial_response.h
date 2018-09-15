#pragma once

#include <Eigen/Eigen>
#include <pcalib/exception.h>
#include <pcalib/response.h>

namespace pcalib
{

class PolynomialResponse : public Response
{
  public:

    PolynomialResponse()
    {
      set_degree(1);
      coeffs_[0] = 1;
    }

    PolynomialResponse(int degree)
    {
      set_degree(degree);
      coeffs_[0] = 1;
    }

    ~PolynomialResponse()
    {
    }

    int degree() const
    {
      return parameters().size();
    }

    void set_degree(int degree)
    {
      PCALIB_ASSERT_MSG(degree >= 1, "degree must be positive");
      const int min_degree = std::min(degree, int(coeffs_.size()));
      std::vector<double> coeffs(degree);
      std::fill(coeffs.begin(), coeffs.end(), 0.0);
      std::copy(coeffs_.begin(), coeffs_.begin() + min_degree, coeffs.begin());
      std::swap(coeffs, coeffs_);
    }

    int parameter_count() const override
    {
      return coeffs_.size();
    }

    std::vector<double> parameters() const override
    {
      return coeffs_;
    }

    void set_parameters(const std::vector<double>& params) override
    {
      PCALIB_ASSERT_MSG(params.size() == coeffs_.size(), "degree mismatch");
      coeffs_ = params;
    }

    const double& operator[](int index) const
    {
      PCALIB_DEBUG(index >= 0 && index < (int)coeffs_.size());
      return coeffs_[index];
    }

    double& operator[](int index)
    {
      PCALIB_DEBUG(index >= 0 && index < (int)coeffs_.size());
      return coeffs_[index];
    }

    inline double operator()(double value) const
    {
      return (*this)(coeffs_.data(), value);
    }

    template <typename Scalar>
    Scalar operator()(const Scalar* parameters, Scalar value) const
    {
      // PCALIB_DEBUG(!std::isnan(value));

      Scalar pow = value;
      Scalar result = Scalar(0);

      for (size_t i = 0; i < coeffs_.size(); ++i)
      {
        result += pow * parameters[i];
        pow *= value;
      }

      return result;
    }

    void Accept(ResponseVisitor& visitor) const override
    {
      visitor.Visit(*this);
    }

    void Reset() override
    {
      std::fill(coeffs_.begin(), coeffs_.end(), 0.0);
      coeffs_[0] = 1.0;
    }

  protected:

    std::vector<double> coeffs_;
};

} // namespace pcalib