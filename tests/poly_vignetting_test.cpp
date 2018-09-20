#include <gtest/gtest.h>
#include <pcalib/poly_vignetting.h>

namespace pcalib
{
namespace testing
{

TEST(EvenPoly6Vignetting, Constructor)
{
  EvenPoly6Vignetting<double> vignetting(640, 480);
}

} // namespace testing

} // namespace pcalib