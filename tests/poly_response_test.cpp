#include <gtest/gtest.h>
#include <pcalib/exception.h>
#include <pcalib/poly_response.h>

namespace pcalib
{
namespace testing
{

TEST(Poly3Response, Constructor)
{
  Poly3Response<double> response;
}

} // namespace testing

} // namespace pcalib