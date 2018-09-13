#pragma once

#include <string>

namespace pcalib
{

class Calibration;

class CalibrationWriter
{
  public:

    CalibrationWriter(const std::string& file);

    void Write(const Calibration& calibration);
};

} // namespace pcalib