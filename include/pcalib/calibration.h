#pragma once

#include <memory>
#include <vector>
#include <pcalib/response.h>
#include <pcalib/vignetting.h>

namespace pcalib
{

struct Calibration
{
  std::vector<std::shared_ptr<Response>> responses;

  std::shared_ptr<Vignetting> vignetting;
};

} // namespace pcalib