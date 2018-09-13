#pragma once

#include <vector>

namespace pcalib
{

class ResponseProblem
{
  public:

  struct Observation
  {
    double intensity;
    double exposure;
  };

  struct Correspondence
  {
    Observation a;
    Observation b;
  };

  std::vector<Correspondence> correspondences;
};

} // namespace pcalib