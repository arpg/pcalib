#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <tinyxml2.h>

namespace pcalib
{

class Calibration;
class Response;
class Vignetting;

class CalibrationWriter
{
  public:

    CalibrationWriter(const std::string& file);

    const std::string& file() const;

    void Write(const Calibration& calibration);

  protected:

    void PrepareWrite();

    void WriteResponses(const Calibration& calibration);

    void WriteVignetting(const Calibration& calibration);

    void FinishWrite();

  protected:

    std::string file_;

    std::shared_ptr<tinyxml2::XMLDocument> document_;
};

} // namespace pcalib