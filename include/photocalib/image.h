#pragma once

#include <opencv2/opencv.hpp>
#include <photocalib/exception.h>

namespace photocalib
{

class Image
{
  public:

    Image() :
      exposure_(0.0)
    {
    }

    Image(const Image& image) :
      data_(image.data().clone()),
      exposure_(image.exposure())
    {
    }

    explicit Image(const cv::Mat& data) :
      data_(data.clone()),
      exposure_(0.0)
    {
    }

    Image& operator=(const cv::Mat& data)
    {
      data_ = data.clone();
      return *this;
    }

    inline int width() const
    {
      return data_.cols;
    }

    inline int height() const
    {
      return data_.rows;
    }

    inline int channels() const
    {
      return data_.channels();
    }

    inline double exposure() const
    {
      return exposure_;
    }

    inline void set_exposure(double exposure)
    {
      PHOTOCALIB_DEBUG(exposure >= 0.0);
      exposure_ = exposure;
    }

    inline double mean(int channel = -1) const
    {
      const cv::Scalar means = cv::mean(data_);
      return (channel < 0) ? cv::sum(means)[0] : means[channel];
    }

    inline const cv::Mat& data() const
    {
      return data_;
    }

    Image operator+(const Image& rhs)
    {
      Image result(*this);
      result += rhs;
      return result;
    }

    Image& operator+=(const Image& rhs)
    {
      PHOTOCALIB_ASSERT_MSG(exposure_ == rhs.exposure(), "exposure mismatch");
      data_ += rhs.data();
      return *this;
    }

    Image operator*(double rhs)
    {
      Image result(*this);
      result *= rhs;
      return result;
    }

    Image& operator*=(double rhs)
    {
      data_ *= rhs;
      return *this;
    }

  protected:

    cv::Mat data_;

    double exposure_;
};

} // namespace photocalib