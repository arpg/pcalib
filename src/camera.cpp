#include <photocalib/camera.h>
#include <photocalib/image.h>

#include <iostream>

namespace photocalib
{

Camera::Camera() :
  auto_exposure_enabled_(false),
  frame_count_(0),
  last_exposure_update_(0)
{
  Initialize();
}

void Camera::Capture(Image &image)
{
  PrepareCapture();
  PerformCapture(image);
  FinishCapture(image);
}

const ExposureTarget& Camera::exposure_target() const
{
  return exposure_target_;
}

void Camera::set_exposure_target(const ExposureTarget& target)
{
  exposure_target_ = target;
  exposure_controller_.set_setpoint(exposure_target_.intensity);
}

double Camera::exposure() const
{
  return raw_exposure();
}

void Camera::set_exposure(double exposure)
{
  auto_exposure_enabled_ = exposure <= 0;
  if (!auto_exposure_enabled_) set_raw_exposure(exposure);
}

double Camera::gain() const
{
  return raw_gain();
}

void Camera::set_gain(double gain)
{
  set_raw_gain(gain);
}

void Camera::PrepareCapture()
{
  ++frame_count_;
}

void Camera::PerformCapture(Image& image)
{
  cv::Mat data;
  Capture(data);
  image = Image(data);
  image.set_exposure(exposure());
}

void Camera::FinishCapture(const Image& image)
{
  if (auto_exposure_enabled_) UpdateExposure(image);
}

void Camera::UpdateExposure(const Image& image)
{
  if (frame_count_ - last_exposure_update_ > exposure_delay())
  {
    const double state = image.exposure();
    const double feedback = image.mean(exposure_target_.channel);
    const double update = exposure_controller_.Update(feedback, state);

    if (std::abs(state - update) >= 0.99)
    {
      last_exposure_update_ = frame_count_;
      set_raw_exposure(update);
    }
  }
}

void Camera::Initialize()
{
  CreateExposureTarget();
  CreateExposureController();
}

void Camera::CreateExposureTarget()
{
  exposure_target_.intensity = 127.5;
  exposure_target_.channel = 0;
}

void Camera::CreateExposureController()
{
  exposure_controller_.set_setpoint(exposure_target_.intensity);

  // orbbec
  // exposure_controller_.set_proportional_gain(0.25);
  // exposure_controller_.set_integral_gain(0.02);

  // // xtion
  // exposure_controller_.set_proportional_gain(0.05);
  // exposure_controller_.set_integral_gain(0.01);

  // exposure_controller_.set_integral_delay(4);
  // exposure_controller_.set_upper_bound(500);
  // exposure_controller_.set_lower_bound(1);

  // realsense2
  exposure_controller_.set_proportional_gain(0.5);
  exposure_controller_.set_integral_gain(0.02);

  exposure_controller_.set_integral_delay(1);
  exposure_controller_.set_upper_bound(10000);
  exposure_controller_.set_lower_bound(1);
}

} // namespace photocalib