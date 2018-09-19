#include <gtest/gtest.h>
#include <pcalib/linear_response.h>

namespace pcalib
{
namespace testing
{

TEST(LinearResponse, Constructor)
{
  LinearResponse<double> response;
  ASSERT_EQ(0, response.NumParams);
}

TEST(LinearResponse, Response)
{
  LinearResponse<double> response;

  for (int i = 0; i < 256; ++i)
  {
    ASSERT_EQ(i, response(i));
  }
}

TEST(LinearResponse, Parameters)
{
  Eigen::VectorXd params(1);
  LinearResponse<double> response;
  ASSERT_THROW(response.SetParams(params), Exception);
}

} // namespace testing

} // namespace pcalib