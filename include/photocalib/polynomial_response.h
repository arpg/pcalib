#pragma once

#include <Eigen/Eigen>
#include <photocalib/exception.h>

namespace photocalib
{

class PolynomialResponse
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
      PHOTOCALIB_ASSERT_MSG(degree >= 1, "degree must be positive");
      const int min_degree = std::min(degree, int(coeffs_.size()));
      Eigen::VectorXd coeffs(degree);
      coeffs.setZero();
      coeffs.head(min_degree) = coeffs_.head(min_degree);
      std::swap(coeffs, coeffs_);
    }

    const Eigen::VectorXd& parameters() const
    {
      return coeffs_;
    }

    void set_parameters(const Eigen::VectorXd& coeffs)
    {
      PHOTOCALIB_ASSERT_MSG(coeffs.size() == coeffs_.size(), "degree mismatch");
      coeffs_ = coeffs;
    }

    const double& operator[](int index) const
    {
      PHOTOCALIB_DEBUG(index >= 0 && index < coeffs_.size());
      return coeffs_[index];
    }

    double& operator[](int index)
    {
      PHOTOCALIB_DEBUG(index >= 0 && index < coeffs_.size());
      return coeffs_[index];
    }

    int parameter_count() const
    {
      return coeffs_.size();
    }

    double operator()(double value) const
    {
      return (*this)(coeffs_.data(), value);
    }

    template <typename Scalar>
    Scalar operator()(const Scalar* parameters, Scalar value) const
    {
      // PHOTOCALIB_DEBUG(!std::isnan(value));

      Scalar pow = value;
      Scalar result = Scalar(0);

      for (int i = 0; i < coeffs_.size(); ++i)
      {
        result += pow * parameters[i];
        pow *= value;
      }

      return result;
    }

  protected:

    Eigen::VectorXd coeffs_;
};

} // namespace photocalib