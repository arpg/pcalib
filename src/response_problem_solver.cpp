#include <pcalib/response_problem_solver.h>
#include <random>
#include <ceres/ceres.h>
#include <Eigen/Cholesky>
#include <pcalib/exception.h>
#include <pcalib/poly_response.h>
#include <pcalib/response_problem.h>

namespace pcalib
{

template <typename Response>
struct ResponseFunctor
{
  ResponseFunctor(std::vector<ResponseProblem::Correspondence>& correspondeces) :
    correspondences(correspondeces)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    const T* coeffs = parameters[0];

    for (size_t i = 0; i < correspondences.size(); ++i)
    {
      const ResponseProblem::Correspondence& correspondence =
          correspondences[i];

      const T int_a = T(correspondence.a.intensity);
      const T int_b = T(correspondence.b.intensity);

      const T irr_a = Response::GetResponse(coeffs, int_a);
      const T irr_b = Response::GetResponse(coeffs, int_b);

      const T exp_a = T(correspondence.a.exposure);
      const T exp_b = T(correspondence.b.exposure);

      const T exp_ratio = exp_a / exp_b;
      residuals[i] = irr_a - exp_ratio * irr_b;
    }

    const T w = T(correspondences.size());
    residuals[correspondences.size()] = w * (T(1) - Response::GetResponse(coeffs, T(1)));
    return true;
  }

  std::shared_ptr<Response> response;

  std::vector<ResponseProblem::Correspondence>& correspondences;
};

template <typename Response>
ResponseProblemSolver<Response>::ResponseProblemSolver(
    std::shared_ptr<const ResponseProblem> problem) :
  problem_(problem),
  inlier_threshold_(0.01),
  ransac_iterations_(10)
{
}

template <typename Response>
std::shared_ptr<const ResponseProblem> ResponseProblemSolver<Response>::problem() const
{
  return problem_;
}

template<typename Response>
double ResponseProblemSolver<Response>::inlier_threshold() const
{
  return inlier_threshold_;
}

template<typename Response>
void ResponseProblemSolver<Response>::set_inlier_threshold(double threshold)
{
  PCALIB_ASSERT_MSG(threshold > 0, "threshold must be positive");
  inlier_threshold_ = threshold;
}

template <typename Response>
int ResponseProblemSolver<Response>::ransac_iterations() const
{
  return ransac_iterations_;
}

template <typename Response>
void ResponseProblemSolver<Response>::set_ransac_iterations(int iterations)
{
  PCALIB_ASSERT_MSG(iterations >= 0, "iteration count cannot be negative");
  ransac_iterations_ = iterations;
}

template <typename Response>
void ResponseProblemSolver<Response>::Solve(std::shared_ptr<Response> response)
{
  ResponseProblem subproblem;
  GetInliers(response, subproblem);
  Solve(response, subproblem);
}

template<typename Response>
void ResponseProblemSolver<Response>::RegisterCallback(const Callback& callback)
{
  callbacks_.push_back(callback);
}

template<typename Response>
void ResponseProblemSolver<Response>::NotifyCallbacks(
    const Response& response) const
{
  for (const Callback& callback : callbacks_)
  {
    if (callback) callback(response);
  }
}

template <typename Response>
void ResponseProblemSolver<Response>::GetInliers(std::shared_ptr<Response> response,
    ResponseProblem& subproblem) const
{
  if (ransac_iterations_ == 0)
  {
    subproblem = *problem_;
    return;
  }

  std::mt19937 rng;
  const int max_index  = problem_->correspondences.size() - 1;
  std::uniform_int_distribution<unsigned int> dist(0, max_index);
  const int param_count = Response::NumParams;

  std::vector<ResponseProblem::Correspondence> inliers;
  inliers.reserve(problem_->correspondences.size());
  int best_inlier_count = 0;

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver;
  Eigen::MatrixXd A(param_count, param_count);
  Eigen::VectorXd b(param_count);

  A.row(param_count - 1).setOnes();
  b.setZero();
  b[param_count - 1] = 1.0;

  for (int i = 0; i < ransac_iterations_; ++i)
  {
    if (i % 10 == 0)
      std::cout << "ransac iteration: " << (i + 1) << "/" << ransac_iterations_ << std::endl;

    std::vector<ResponseProblem::Correspondence> correspondences(param_count - 1);

    for (size_t j = 0; j < correspondences.size(); ++j)
    {
      const int index = dist(rng);
      correspondences[j] = problem_->correspondences[index];
    }

    Eigen::VectorXd x = response->GetParams();

    for (size_t i = 0; i < correspondences.size(); ++i)
    {
      const double ea = correspondences[i].a.exposure;
      const double eb = correspondences[i].b.exposure;
      const double r = ea / eb;

      const double ia = correspondences[i].a.intensity;
      const double ib = correspondences[i].b.intensity;

      for (int j = 0; j < x.size(); ++j)
      {
        A(i, j) = std::pow(ia, j + 1) - r * std::pow(ib, j + 1);
      }
    }

    solver.compute(A);
    if (solver.info() != Eigen::Success) continue;
    x = solver.solve(b);
    if (!(A * x).isApprox(b, 1E-6)) continue;

    Response new_response;
    new_response.SetParams(x);
    inliers.clear();

    for (const ResponseProblem::Correspondence& c : problem_->correspondences)
    {
      const double int_a = c.a.intensity;
      const double int_b = c.b.intensity;

      const double irr_a = new_response(int_a);
      const double irr_b = new_response(int_b);

      const double exp_a = c.a.exposure;
      const double exp_b = c.b.exposure;

      const double irr_ratio = irr_a / irr_b;
      const double exp_ratio = exp_a / exp_b;

      const double error = irr_ratio - exp_ratio;

      if (std::abs(error) < inlier_threshold_)
      {
        inliers.push_back(c);
      }
    }

    const int inlier_count = inliers.size();

    if (inlier_count > best_inlier_count)
    {
      best_inlier_count = inlier_count;
      subproblem.correspondences = inliers;
      *response = new_response;

      std::cout << "new inlier count: " << inliers.size() << " (" <<
          (100.0 * inliers.size() / problem_->correspondences.size()) << "%)" << std::endl;

      NotifyCallbacks(*response);
    }
  }

  std::cout << "final inlier count: " << inliers.size() << " (" <<
      (100.0 * subproblem.correspondences.size() / problem_->correspondences.size()) << "%)" << std::endl;
}

template <typename Response>
void ResponseProblemSolver<Response>::Solve(std::shared_ptr<Response> response,
    ResponseProblem& subproblem)
{
  Eigen::VectorXd parameters = response->GetParams();

  ceres::Problem problem;
  problem.AddParameterBlock(parameters.data(), parameters.size());

  ResponseFunctor<Response>* functor;
  functor = new ResponseFunctor<Response>(subproblem.correspondences);
  functor->response = response;

  ceres::DynamicAutoDiffCostFunction<ResponseFunctor<Response>>* cost;
  cost = new ceres::DynamicAutoDiffCostFunction<ResponseFunctor<Response>>(functor);
  cost->AddParameterBlock(parameters.size());
  cost->SetNumResiduals(subproblem.correspondences.size() + 1);

  ceres::SoftLOneLoss* loss = new ceres::SoftLOneLoss(0.5);
  problem.AddResidualBlock(cost, loss, parameters.data());

  ceres::Solver::Options options;
  options.minimizer_type = ceres::TRUST_REGION;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;

  Response new_response;
  new_response.SetParams(parameters);
  *response = new_response;
}

template class ResponseProblemSolver<Poly3Response<double>>;
template class ResponseProblemSolver<Poly4Response<double>>;

} // namespace pcalib