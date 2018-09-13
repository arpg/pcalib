#pragma once

#include <memory>
#include <pcalib/light.h>

namespace pcalib
{

class LightProblem;

class LightProblemSolver
{
  public:

    LightProblemSolver(std::shared_ptr<const LightProblem> problem);

    std::shared_ptr<const LightProblem> problem() const;

    double inlier_threshold() const;

    void set_inlier_threshold(double threshold);

    int ransac_iterations() const;

    void set_ransac_iterations(int iterations);

    void Solve(Light<double>& light);

  protected:

    void GetSubproblem(LightProblem& problem, Light<double>& light);

    void SolveProblem(const LightProblem& problem, Light<double>& light);

  protected:

    std::shared_ptr<const LightProblem> problem_;

    double inlier_threshold_;

    int ransac_iterations_;
};

} // namespace pcalib