#pragma once

#include <memory>
#include <string>

namespace photocalib
{

class Calibration;
class Response;
class Vignetting;

class CalibrationReader
{
  public:

    CalibrationReader(const std::string& file);

    void Read(Calibration& calibration);
};

} // namespace photocalib