#include <pcalib/calibration_writer.h>
#include <sstream>
#include <pcalib/calibration.h>
#include <pcalib/exception.h>
#include <pcalib/polynomial_response.h>
#include <pcalib/vignetting.h>
#include <pcalib/visitor.h>

namespace pcalib
{

class Element
{
  public:

    Element(tinyxml2::XMLElement* element) :
      element_(element)
    {
    }

    inline tinyxml2::XMLElement* get() const
    {
      return element_;
    }

    virtual void SetType(const std::string& type)
    {
      element_->SetAttribute("type", type.c_str());
    }

    virtual void SetParameters(const std::vector<double>& params)
    {
      tinyxml2::XMLDocument* document = element_->GetDocument();
      tinyxml2::XMLElement* child = document->NewElement("params");
      child->SetText(ToString(params).c_str());
      element_->InsertEndChild(child);
    }

  protected:

    virtual std::string ToString(const std::vector<double>& values)
    {
      std::stringstream text;
      text << " [ ";

      for (size_t i = 0; i < values.size() - 1; ++i)
      {
        text << values[i] << "; ";
      }

      if (!values.empty())
      {
        text << values.back();
      }

      text << " ] ";
      return text.str().c_str();
    }

  protected:

    tinyxml2::XMLElement* element_;
};

class ResponseElement : public Element
{
  public:

    ResponseElement(tinyxml2::XMLElement* element) :
      Element(element)
    {
    }

    void SetChannel(int channel)
    {
      element_->SetAttribute("channel", channel);
    }
};

class VignettingElement : public Element
{
  public:

    VignettingElement(tinyxml2::XMLElement* element) :
      Element(element)
    {
    }

    void SetType(const std::string& type)
    {
      element_->SetAttribute("type", type.c_str());
    }
};

class ResponseWriter : public ResponseVisitor
{
  public:

    ResponseWriter(std::shared_ptr<tinyxml2::XMLDocument> document) :
      document_(document),
      channel_(0)
    {
    }

    inline int channel() const
    {
      return channel_;
    }

    inline void set_channel(int channel)
    {
      channel_ = channel;
    }

    void Write(const Response& response)
    {
      response.Accept(*this);
    }

    void Visit(const LinearResponse& response) override
    {
      ResponseElement element = CreateElement(response);
      element.SetType("linear");
    }

    void Visit(const PolynomialResponse& response) override
    {
      ResponseElement element = CreateElement(response);
      tinyxml2::XMLElement* degree = document_->NewElement("degree");
      degree->SetText(response.degree());
      element.get()->InsertFirstChild(degree);
      element.SetType("poly");
    }

  protected:

    ResponseElement CreateElement(const Response& response)
    {
      tinyxml2::XMLElement* root = document_->FirstChildElement("pcalib");
      ResponseElement element(document_->NewElement("response"));
      root->InsertEndChild(element.get());

      if (response.parameter_count() > 0)
      {
        element.SetParameters(response.parameters());
      }

      element.SetChannel(channel_);
      return element;
    }

  protected:

    std::shared_ptr<tinyxml2::XMLDocument> document_;

    int channel_;
};

class VignettingWriter : public VignettingVisitor
{
  public:

    VignettingWriter(std::shared_ptr<tinyxml2::XMLDocument> document) :
      document_(document)
    {
    }

    void Write(const Vignetting& vignetting)
    {
      vignetting.Accept(*this);

      // tinyxml2::XMLElement* file = document_->NewElement("file");
      // file->SetText("/home/mike/vignetting.png");
      // element.get()->InsertEndChild(file);
    }

    void Visit(const TrivialVignetting& vignetting) override
    {
      VignettingElement element = CreateElement(vignetting);
      element.SetType("none");
    }

  protected:

    VignettingElement CreateElement(const Vignetting& vignetting)
    {
      tinyxml2::XMLElement* root = document_->FirstChildElement("pcalib");
      VignettingElement element(document_->NewElement("vignetting"));
      root->InsertEndChild(element.get());

      if (vignetting.parameter_count() > 0)
      {
        element.SetParameters(vignetting.parameters());
      }

      return element;
    }

  protected:

    std::shared_ptr<tinyxml2::XMLDocument> document_;
};

CalibrationWriter::CalibrationWriter(const std::string& file) :
  file_(file)
{
}

const std::string& CalibrationWriter::file() const
{
  return file_;
}

void CalibrationWriter::Write(const Calibration& calibration)
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

void CalibrationWriter::WriteResponses(const Calibration& calibration)
{
  ResponseWriter writer(document_);

  for (size_t i = 0; i < calibration.responses.size(); ++i)
  {
    writer.set_channel(i);
    writer.Write(*calibration.responses[i]);
  }
}

void CalibrationWriter::WriteVignetting(const Calibration& calibration)
{
  if (calibration.vignetting)
  {
    VignettingWriter writer(document_);
    writer.Write(*calibration.vignetting);
  }
}

void CalibrationWriter::FinishWrite()
{
  document_->SaveFile(file_.c_str());
  document_.reset();
}

} // namespace pcalib