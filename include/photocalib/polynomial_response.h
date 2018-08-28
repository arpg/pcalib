#pragma once

#include <Eigen/Eigen>
#include <photocalib/response.h>

namespace photocalib
{

class PolynomialResponse : public Response
{
  public:

    PolynomialResponse();

    PolynomialResponse(int degree);

    int degree() const;

    void set_degree(int degree);

    const Eigen::VectorXd& coeffs() const;

    void set_coeffs(const Eigen::VectorXd& coeffs);

    double operator()(double value) const override;

    const double& operator[](int index) const;

    double& operator[](int index);

  protected:

    Eigen::VectorXd coeffs_;
};

} // namespace photocalib