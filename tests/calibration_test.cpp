#include <gtest/gtest.h>
#include <pcalib/calibration.h>

namespace pcalib
{
namespace testing
{

TEST(Calibration, Constructor)
{
  Calibration<double> calibration;
}

} // namespace testing

} // namespace pcalib