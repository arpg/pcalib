#pragma once

#include <vector>

namespace pcalib
{

class Image;
class ResponseProblem;

class ResponseProblemBuilder
{
  public:

    ResponseProblemBuilder();

    bool join_channels() const;

    void set_join_channels(bool join);

    void AddImage(const Image& image);

    void Build(std::vector<ResponseProblem>& problems);

  protected:

    void Build(int channel, ResponseProblem& problem);

  protected:

    bool join_channels_;

    std::vector<Image> images_;
};

} // namespace pcalib