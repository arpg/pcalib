#pragma once

#include <memory>

namespace photocalib
{

class Response;
class Vignetting;

class Calibration
{
  public:

    Calibration();

  protected:

    std::shared_ptr<Response> responses_;

    std::shared_ptr<Vignetting> vignetting_;
};

} // namespace photocalib