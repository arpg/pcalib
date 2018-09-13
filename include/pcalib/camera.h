#pragma once

#include <pcalib/image.h>
#include <pcalib/controller.h>

namespace pcalib
{

class Image;

struct ExposureTarget
{
  ExposureTarget() :
    intensity(0.5),
    channel(0)
  {
  }

  ExposureTarget(double intensity, int channel = -1) :
    intensity(intensity),
    channel(channel)
  {
  }

  double intensity;

  int channel;
};

class Camera
{
  public:

    Camera();

    virtual void Capture(Image& image);

    virtual const ExposureTarget& exposure_target() const;

    virtual void set_exposure_target(const ExposureTarget& target);

    virtual double exposure() const;

    virtual void set_exposure(double time);

    virtual double gain() const;

    virtual void set_gain(double time);

  protected:

    void PrepareCapture();

    void PerformCapture(Image& image);

    void FinishCapture(const Image& image);

    void UpdateExposure(const Image& image);

    virtual void Capture(cv::Mat& data) = 0;

    virtual double raw_exposure() const = 0;

    virtual void set_raw_exposure(double exposure) = 0;

    virtual double raw_gain() const = 0;

    virtual void set_raw_gain(double gain) = 0;

    virtual int exposure_delay() const = 0;

  private:

    void Initialize();

    void CreateExposureTarget();

    void CreateExposureController();

  private:

    double target_;

    bool auto_exposure_enabled_;

    Controller exposure_controller_;

    ExposureTarget exposure_target_;

    int frame_count_;

    int last_exposure_update_;
};

} // namespace pcalib