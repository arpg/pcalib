#pragma once

#include <memory>
#include <string>
#include <fstream>
#include <tinyxml2.h>
#include <pcalib/calibration.h>

namespace pcalib
{

class Vignetting;

class CalibrationWriter
{
  public:

    CalibrationWriter(const std::string& file);

    const std::string& file() const;

    void Write(const Calibration<double>& calibration);

  protected:

    void PrepareWrite();

    void WriteResponses(const Calibration<double>& calibration);

    void WriteResponse(int channel, const Response<double>& response);

    void WriteVignetting(const Calibration<double>& calibration);

    void WriteVignetting(const Vignetting& vignetting);

    void WriteData(tinyxml2::XMLElement* element, const Eigen::MatrixXd& value);

    void FinishWrite();

  protected:

    std::string file_;

    std::shared_ptr<tinyxml2::XMLDocument> document_;
};

} // namespace pcalib