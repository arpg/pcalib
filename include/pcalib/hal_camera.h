#pragma once

#include <memory>
#include <pcalib/camera.h>

namespace hal
{

class Camera;
class OpenNI2Driver;
class RealSense2Driver;

} // namespace hal

namespace pcalib
{

class HalCamera : public Camera
{
  public:

    HalCamera(const std::string& uri);

    virtual ~HalCamera();

  protected:

    void Capture(cv::Mat& data) override;

    double raw_exposure() const override;

    void set_raw_exposure(double exposure) override;

    double raw_gain() const override;

    void set_raw_gain(double gain) override;

    int exposure_delay() const override;

  private:

    void Initialize();

    void CreateDriver();

    void PrepareCapture();

  protected:

    std::unique_ptr<hal::Camera> camera_;

    // hal::OpenNI2Driver* driver_;

    hal::RealSense2Driver* driver_;
};

} // namespace pcalib