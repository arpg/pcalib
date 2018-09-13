#include <pcalib/polynomial_response.h>
#include <pcalib/exception.h>

namespace pcalib
{

// PolynomialResponse::PolynomialResponse()
// {
//   set_degree(1);
//   coeffs_[0] = 1;
// }
//
// PolynomialResponse::PolynomialResponse(int degree)
// {
//   set_degree(degree);
//   coeffs_[0] = 1;
// }
//
// PolynomialResponse::~PolynomialResponse()
// {
// }
//
// int PolynomialResponse::degree() const
// {
//   return parameters().size();
// }
//
// void PolynomialResponse::set_degree(int degree)
// {
//   PHOTOCALIB_ASSERT_MSG(degree >= 1, "degree must be positive");
//   const int min_degree = std::min(degree, int(coeffs_.size()));
//   Eigen::VectorXd coeffs(degree);
//   coeffs.setZero();
//   coeffs.head(min_degree) = coeffs_.head(min_degree);
//   std::swap(coeffs, coeffs_);
// }
//
// const PolynomialResponse::Parameters& PolynomialResponse::parameters() const
// {
//   return coeffs_;
// }
//
// void PolynomialResponse::set_parameters(const Eigen::VectorXd& coeffs)
// {
//   PHOTOCALIB_ASSERT_MSG(coeffs.size() == coeffs_.size(), "degree mismatch");
//   coeffs_ = coeffs;
// }
//
// // double PolynomialResponse::operator()(double value) const
// // {
// //   PHOTOCALIB_DEBUG(!std::isnan(value));
// //
// //   double pow = value;
// //   double result = 0;
// //
// //   for (int i = 0; i < coeffs_.size(); ++i)
// //   {
// //     result += pow * coeffs_[i];
// //     pow *= value;
// //   }
// //
// //   return result;
// // }
//
// const double& PolynomialResponse::operator[](int index) const
// {
//   PHOTOCALIB_DEBUG(index >= 0 && index < coeffs_.size());
//   return coeffs_[index];
// }
//
// double& PolynomialResponse::operator[](int index)
// {
//   PHOTOCALIB_DEBUG(index >= 0 && index < coeffs_.size());
//   return coeffs_[index];
// }
//
// int PolynomialResponse::parameter_count() const
// {
//   return coeffs_.size();
// }

} // namespace pcalib