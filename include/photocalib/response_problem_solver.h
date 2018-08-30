#pragma once

#include <memory>

namespace photocalib
{

class Response;
class ResponseProblem;

template <typename Response>
class ResponseProblemSolver
{
  public:

    ResponseProblemSolver(std::shared_ptr<const ResponseProblem> problem);

    std::shared_ptr<const ResponseProblem> problem() const;

    double inlier_threshold() const;

    void set_inlier_threshold(double threshold);

    int ransac_iterations() const;

    void set_ransac_iterations(int iterations);

    void Solve(std::shared_ptr<Response> response);

  protected:

    void GetInliers(std::shared_ptr<Response> response,
        ResponseProblem& subproblem) const;

    void Solve(std::shared_ptr<Response> response,
        ResponseProblem& subproblem);

  protected:

    std::shared_ptr<const ResponseProblem> problem_;

    double inlier_threshold_;

    int ransac_iterations_;
};

} // namespace photocalib