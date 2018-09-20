#include <gtest/gtest.h>
#include <pcalib/calibration_writer.h>
#include <pcalib/calibration.h>
#include <pcalib/poly_response.h>
#include <pcalib/linear_response.h>

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
  Calibration<double> calibration;

  for (int i = 0; i < 3; ++i)
  {
    std::shared_ptr<Poly3Response<double>> response;
    response = std::make_shared<Poly3Response<double>>();
    response->SetParams((i + 1) * Eigen::Vector3d(0.2, 1.1, 2.0));
    calibration.responses.push_back(response);
  }

  std::shared_ptr<LinearResponse<double>> response;
  response = std::make_shared<LinearResponse<double>>();
  calibration.responses.push_back(response);

  // std::shared_ptr<TrivialVignetting> vignetting;
  // vignetting = std::make_shared<TrivialVignetting>();
  // calibration.vignetting = vignetting;

  const std::string file = "test.xml";
  CalibrationWriter writer(file);
  writer.Write(calibration);
}

} // namespace testing

} // namespace pcalib