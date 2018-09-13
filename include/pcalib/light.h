#pragma once

#include <Eigen/Eigen>
#include <pcalib/exception.h>

namespace pcalib
{

template <typename T>
class Light
{
  public:

    typedef Eigen::Matrix<T, 3, 1> Vector3t;

  public:

    Light() :
      intensity_(T(1)),
      position_(Vector3t::Zero())
    {
    }

    inline T intensity() const
    {
      return intensity_;
    }

    inline void set_intensity(T intensity)
    {
      PHOTOCALIB_DEBUG_MSG(intensity > T(0), "intensity must be positive");
      intensity_ = intensity;
    }

    inline const Vector3t& position() const
    {
      return position_;
    }

    inline void set_position(const Vector3t& position)
    {
      position_ = position;
    }

    inline void set_position(T x, T y, T z)
    {
      set_position(Vector3t(x, y, z));
    }

    inline T GetShading(const Vector3t& p, const Vector3t& n) const
    {
      const Vector3t delta = position_ - p;
      const T distance_squared = delta.dot(delta);
      const Vector3t direction = delta.normalized();
      const T cos_theta = direction.dot(n);
      return intensity_ * cos_theta / distance_squared;
    }

  protected:

    T intensity_;

    Vector3t position_;
};

} // namespace pcalib