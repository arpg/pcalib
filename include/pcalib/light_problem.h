#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <pcalib/light.h>

namespace pcalib
{

struct LightSample
{
  double irradiance;

  Eigen::Vector3d Xcp;

  Eigen::Vector3d Xcn;
};

struct LightProblem
{
  std::vector<LightSample> samples;
};

} // namespace pcalib