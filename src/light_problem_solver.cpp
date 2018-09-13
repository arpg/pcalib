#include <pcalib/light_problem_solver.h>
#include <random>
#include <ceres/ceres.h>
#include <pcalib/exception.h>
#include <pcalib/light_problem.h>

#include <iostream>

namespace pcalib
{

struct LightFunctor
{
  LightFunctor(const std::vector<LightSample>& samples) :
    samples(samples)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    Light<T> light;
    light.set_intensity(parameters[0][0]);
    light.set_position(parameters[1][0], parameters[1][1], parameters[1][2]);

    for (size_t i = 0; i < samples.size(); ++i)
    {
      const typename Light<T>::Vector3t p = samples[i].Xcp.cast<T>();
      const typename Light<T>::Vector3t n = samples[i].Xcn.cast<T>();
      const T expected = light.GetShading(p, n);
      const T found = T(samples[i].irradiance);
      residuals[i] = expected - found;
    }

    return true;
  }

  const std::vector<LightSample> samples;
};

LightProblemSolver::LightProblemSolver(
    std::shared_ptr<const LightProblem> problem) :
  problem_(problem),
  inlier_threshold_(0.01),
  ransac_iterations_(10)
{
}

std::shared_ptr<const LightProblem> LightProblemSolver::problem() const
{
  return problem_;
}

double LightProblemSolver::inlier_threshold() const
{
  return inlier_threshold_;
}

void LightProblemSolver::set_inlier_threshold(double threshold)
{
  PCALIB_ASSERT_MSG(threshold > 0, "threshold must be positive");
  inlier_threshold_ = threshold;
}

int LightProblemSolver::ransac_iterations() const
{
  return ransac_iterations_;
}

void LightProblemSolver::set_ransac_iterations(int iterations)
{
  PCALIB_ASSERT_MSG(iterations >= 0, "iteration count cannot be negative");
  ransac_iterations_ = iterations;
}

void LightProblemSolver::Solve(Light<double>& light)
{
  LightProblem subproblem;
  GetSubproblem(subproblem, light);
  SolveProblem(subproblem, light);
}

void LightProblemSolver::GetSubproblem(LightProblem& problem,
    Light<double>& light)
{
  if (ransac_iterations_ == 0)
  {
    problem = *problem_;
    return;
  }

  std::mt19937 rng;
  const int max_index = problem_->samples.size() - 1;
  std::uniform_int_distribution<unsigned int> dist(0, max_index);
  const int param_count = 5;;

  std::vector<LightSample> inliers;
  inliers.reserve(problem_->samples.size());
  int best_inlier_count = 0;

  for (int i = 0; i < ransac_iterations_; ++i)
  {
    if (i % 10 == 0)
      std::cout << "ransac iteration: " << (i + 1) << "/" << ransac_iterations_ << std::endl;

    std::vector<LightSample> samples(param_count);

    for (size_t j = 0; j < samples.size(); ++j)
    {
      const int index = dist(rng);
      samples[j] = problem_->samples[index];
    }

    double intensity = light.intensity();
    Eigen::Vector3d position = light.position();

    ceres::Problem ceres_problem;
    ceres_problem.AddParameterBlock(&intensity, 1);
    ceres_problem.AddParameterBlock(position.data(), 3);

    LightFunctor* functor;
    functor = new LightFunctor(samples);

    ceres::DynamicAutoDiffCostFunction<LightFunctor>* cost;
    cost = new ceres::DynamicAutoDiffCostFunction<LightFunctor>(functor);
    cost->AddParameterBlock(1);
    cost->AddParameterBlock(3);
    cost->SetNumResiduals(samples.size());

    ceres_problem.AddResidualBlock(cost, nullptr, &intensity, position.data());

    // // TODO: remove
    // ceres_problem.SetParameterBlockConstant(position.data());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_type = ceres::TRUST_REGION;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &ceres_problem, &summary);
    if (summary.termination_type == ceres::FAILURE) continue;

    if (intensity <= 0) continue;

    Light<double> ransac_light;
    ransac_light.set_intensity(intensity);
    ransac_light.set_position(position);

    inliers.clear();

    for (const LightSample& sample : problem_->samples)
    {
      const Eigen::Vector3d p = sample.Xcp;
      const Eigen::Vector3d n = sample.Xcn;
      const double expected = ransac_light.GetShading(p, n);
      const double found = sample.irradiance;
      const double error = expected - found;

      if (std::abs(error) < inlier_threshold_)
      {
        inliers.push_back(sample);
      }
    }

    const int inlier_count = inliers.size();

    if (inlier_count > best_inlier_count)
    {
      best_inlier_count = inlier_count;
      problem.samples = inliers;
      light = ransac_light;

      std::cout << "new inlier count: " << inliers.size() << " (" <<
          (100.0 * inliers.size() / problem_->samples.size()) << "%)" << std::endl;
    }
  }

  std::cout << "final inlier count: " << inliers.size() << " (" <<
      (100.0 * problem.samples.size() / problem_->samples.size()) << "%)" << std::endl;
}

void LightProblemSolver::SolveProblem(const LightProblem& problem,
    Light<double>& light)
{
  double intensity = light.intensity();
  Eigen::Vector3d position = light.position();

  ceres::Problem ceres_problem;
  ceres_problem.AddParameterBlock(&intensity, 1);
  ceres_problem.AddParameterBlock(position.data(), 3);

  LightFunctor* functor;
  functor = new LightFunctor(problem.samples);

  ceres::DynamicAutoDiffCostFunction<LightFunctor>* cost;
  cost = new ceres::DynamicAutoDiffCostFunction<LightFunctor>(functor);
  cost->AddParameterBlock(1);
  cost->AddParameterBlock(3);
  cost->SetNumResiduals(problem.samples.size());

  // // TODO: remove
  // ceres_problem.SetParameterBlockConstant(position.data());

  ceres::SoftLOneLoss* loss = new ceres::SoftLOneLoss(0.5);

  ceres_problem.AddResidualBlock(cost, loss, &intensity, position.data());

  ceres::Solver::Options options;
  options.minimizer_type = ceres::TRUST_REGION;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &ceres_problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  light.set_intensity(intensity);
  light.set_position(position);
}

} // namespace pcalib