#pragma once

#include <string>
#include <pcalib/calibration.h>

namespace pcalib
{

class CalibrationReader
{
  public:

    CalibrationReader(const std::string& file);

    void Read(Calibration<double>& calibration);
};

} // namespace pcalib