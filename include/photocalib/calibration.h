#pragma once

#include <memory>
#include <vector>
#include <photocalib/response.h>
#include <photocalib/vignetting.h>

namespace photocalib
{

struct Calibration
{
  std::vector<std::shared_ptr<Response>> responses;

  std::shared_ptr<Vignetting> vignetting;
};

} // namespace photocalib