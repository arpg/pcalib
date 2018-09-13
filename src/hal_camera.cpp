#include <pcalib/hal_camera.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Camera/Drivers/OpenNI2/OpenNI2Driver.h>
#include <HAL/Camera/Drivers/RealSense2/RealSense2Driver.h>
#include <pcalib/exception.h>
#include <pcalib/image.h>

namespace pcalib
{

HalCamera::HalCamera(const std::string& uri) :
  camera_(std::make_unique<hal::Camera>(uri)),
  driver_(nullptr)
{
  Initialize();
}

HalCamera::~HalCamera()
{
}

double HalCamera::raw_exposure() const
{
  return driver_->Exposure();
}

void HalCamera::set_raw_exposure(double exposure)
{
  driver_->SetExposure(exposure);
}

double HalCamera::raw_gain() const
{
  return driver_->Gain();
}

void HalCamera::set_raw_gain(double gain)
{
  driver_->SetGain(gain);
}

int HalCamera::exposure_delay() const
{
  return 3;
}

void HalCamera::Capture(cv::Mat& data)
{
  std::vector<cv::Mat> images;
  PCALIB_DEBUG_MSG(camera_->Capture(images), "failed to capture image");
  PCALIB_DEBUG_MSG(images.size() == 1, "invalid camera stream count");
  data = images[0];
}

void HalCamera::Initialize()
{
  CreateDriver();
  PrepareCapture();
}

void HalCamera::CreateDriver()
{
  // typedef hal::OpenNI2Driver Type;
  typedef hal::RealSense2Driver Type;
  driver_ = dynamic_cast<Type*>(camera_->GetDriver());
  if (!driver_) driver_ = dynamic_cast<Type*>(camera_->GetInputDevice().get());
  PCALIB_ASSERT_MSG(driver_, "openni2 camera required");
}

void HalCamera::PrepareCapture()
{
  std::vector<cv::Mat> images;
  camera_->Capture(images);
  camera_->Capture(images);
  camera_->Capture(images);
  camera_->Capture(images);
  camera_->Capture(images);
  camera_->Capture(images);
}

} // namespace pcalib