#include <pcalib/calibration_writer.h>
#include <sstream>
#include <pcalib/calibration.h>
#include <pcalib/exception.h>
#include <pcalib/poly_response.h>
#include <pcalib/vignetting.h>
#include <pcalib/visitor.h>

namespace pcalib
{

CalibrationWriter::CalibrationWriter(const std::string& file) :
  file_(file)
{
}

const std::string& CalibrationWriter::file() const
{
  return file_;
}

void CalibrationWriter::Write(const Calibration<double>& calibration)
{
  PrepareWrite();
  WriteResponses(calibration);
  WriteVignetting(calibration);
  FinishWrite();
}

void CalibrationWriter::PrepareWrite()
{
  document_ = std::make_shared<tinyxml2::XMLDocument>();
  tinyxml2::XMLElement* root = document_->NewElement("pcalib");
  document_->InsertEndChild(root);
}

void CalibrationWriter::WriteResponses(const Calibration<double>& calibration)
{
  for (size_t i = 0; i < calibration.responses.size(); ++i)
  {
    WriteResponse(i, *calibration.responses[i]);
  }
}

void CalibrationWriter::WriteResponse(int channel,
    const Response<double>& response)
{
  tinyxml2::XMLElement* root = document_->FirstChildElement("pcalib");
  tinyxml2::XMLElement* element = document_->NewElement("response");
  element->SetAttribute("channel", channel);
  element->SetAttribute("type", response.Type().c_str());
  root->InsertEndChild(element);

  tinyxml2::XMLElement* relement = document_->NewElement("range");
  element->InsertEndChild(relement);
  WriteData(relement, response.GetRange());

  tinyxml2::XMLElement* pelement = document_->NewElement("params");
  element->InsertEndChild(pelement);

  if (response.NumParams() > 0)
  {
    WriteData(pelement, response.GetParams());
  }
}

void CalibrationWriter::WriteVignetting(const Calibration<double>& calibration)
{
  for (size_t i = 0; i < calibration.vignetting.size(); ++i)
  {
    WriteVignetting(i, *calibration.vignetting[i]);
  }
}

void CalibrationWriter::WriteVignetting(int channel,
    const Vignetting<double>& vignetting)
{
  tinyxml2::XMLElement* root = document_->FirstChildElement("pcalib");
  tinyxml2::XMLElement* element = document_->NewElement("vignetting");
  element->SetAttribute("channel", channel);
  element->SetAttribute("type", vignetting.Type().c_str());
  root->InsertEndChild(element);

  tinyxml2::XMLElement* welement = document_->NewElement("width");
  element->InsertEndChild(welement);
  welement->SetText(vignetting.Width());

  tinyxml2::XMLElement* helement = document_->NewElement("height");
  element->InsertEndChild(helement);
  helement->SetText(vignetting.Width());

  tinyxml2::XMLElement* pelement = document_->NewElement("params");
  element->InsertEndChild(pelement);

  if (vignetting.NumParams() > 0)
  {
    WriteData(pelement, vignetting.GetParams());
  }
}

void CalibrationWriter::WriteData(tinyxml2::XMLElement* element,
    const Eigen::MatrixXd& value)
{
  std::stringstream data;
  data << " [ ";

  for (int y = 0; y < value.rows(); ++y)
  {
    for (int x = 0; x < value.cols() - 1; ++x)
    {
      data << value(y, x) << ", ";
    }

    data << value(y, value.cols() - 1);
    if (y < value.rows() - 1) data << "; ";
  }

  data << " ] ";
  element->SetText(data.str().c_str());
}

void CalibrationWriter::FinishWrite()
{
  document_->SaveFile(file_.c_str());
  document_.reset();
}

} // namespace pcalib