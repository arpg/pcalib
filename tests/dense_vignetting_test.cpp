#include <gtest/gtest.h>
#include <pcalib/dense_vignetting.h>

namespace pcalib
{
namespace testing
{

TEST(DenseVignetting, Constructor)
{
  DenseVignetting<double> vignetting(640, 480);
}

} // namespace testing

} // namespace pcalib