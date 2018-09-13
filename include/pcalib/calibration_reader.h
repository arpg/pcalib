#pragma once

#include <string>

namespace pcalib
{

class Calibration;

class CalibrationReader
{
  public:

    CalibrationReader(const std::string& file);

    void Read(Calibration& calibration);
};

} // namespace pcalib