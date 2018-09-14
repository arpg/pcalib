#include <gtest/gtest.h>
#include <pcalib/calibration_writer.h>
#include <pcalib/calibration.h>
#include <pcalib/polynomial_response.h>

namespace pcalib
{
namespace testing
{

TEST(CalibrationWriter, Constructor)
{
  const std::string expected_file = "my_file.xml";
  CalibrationWriter writer(expected_file);
  ASSERT_EQ(expected_file, writer.file());
}

TEST(CalibrationWriter, Write)
{
  Calibration calibration;

  for (int i = 0; i < 3; ++i)
  {
    // std::shared_ptr<PolynomialResponse> response;
    // response = std::make_shared<PolynomialResponse>(3);
    // response->set_parameters((i + 1) * Eigen::Vector3d(0.2, 1.1, 2.0));
    // calibration.responses.push_back(response);

    std::shared_ptr<LinearResponse> response;
    response = std::make_shared<LinearResponse>();
    calibration.responses.push_back(response);
  }

  std::shared_ptr<TrivialVignetting> vignetting;
  vignetting = std::make_shared<TrivialVignetting>();
  calibration.vignetting = vignetting;

  const std::string file = "test.xml";
  CalibrationWriter writer(file);
  writer.Write(calibration);
}

} // namespace testing

} // namespace pcalib