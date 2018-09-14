#include <pcalib/response_problem_builder.h>
#include <Eigen/Eigen>
#include <pcalib/exception.h>
#include <pcalib/image.h>
#include <pcalib/response_problem.h>

namespace pcalib
{

ResponseProblemBuilder::ResponseProblemBuilder()
{
}

bool ResponseProblemBuilder::join_channels() const
{
  return join_channels_;
}

void ResponseProblemBuilder::set_join_channels(bool join)
{
  join_channels_ = join;
}

void ResponseProblemBuilder::AddImage(const Image& image)
{
  PCALIB_ASSERT_MSG(image.exposure() > 0, "invalid image exposure");

  if (!images_.empty())
  {
    const Image& front = images_.front();
    PCALIB_ASSERT_MSG(front.width() == image.width(), "size mismatch");
    PCALIB_ASSERT_MSG(front.height() == image.height(), "size mismatch");
  }

  images_.push_back(image);
}

void ResponseProblemBuilder::Build(std::vector<ResponseProblem>& problems)
{
  PCALIB_ASSERT_MSG(images_.size() > 1, "insufficient image count");

  if (join_channels_)
  {
    problems.resize(1);
    Build(-1, problems.front());
  }
  else
  {
    problems.resize(3);

    for (size_t i = 0; i < problems.size(); ++i)
    {
      Build(i, problems[i]);
    }
  }
}

void ResponseProblemBuilder::Build(int channel, ResponseProblem& problem)
{
  problem.correspondences.clear();
  ResponseProblem::Correspondence correspondence;

  for (size_t i = 0; i < images_.size(); ++i)
  {
    const Image& image_i = images_[i];
    correspondence.a.exposure = image_i.exposure();

    for (size_t j = i + 1; j < images_.size(); ++j)
    {
      const Image& image_j = images_[j];
      correspondence.b.exposure = image_j.exposure();

      for (int y = 0; y < image_i.height(); y += 4)
      {
        for (int x = 0; x < image_i.height(); x += 4)
        {
          const Eigen::Vector3f a = image_i.data().at<Eigen::Vector3f>(y, x);
          const Eigen::Vector3f b = image_j.data().at<Eigen::Vector3f>(y, x);
          const bool swap = image_i.exposure() > image_j.exposure();
          const Eigen::Vector3f sa = swap ? b : a;
          const Eigen::Vector3f sb = swap ? a : b;

          if ((channel < 0 || channel == 0) && sa[0] < sb[0] &&
              a[0] > 0.01 && a[0] < 0.99 && b[0] > 0.01 && b[0] < 0.99)
          {
            correspondence.a.intensity = a[0];
            correspondence.b.intensity = b[0];
            problem.correspondences.push_back(correspondence);
          }

          if ((channel < 0 || channel == 1) && sa[1] < sb[1] &&
              a[1] > 0.01 && a[1] < 0.99 && b[1] > 0.01 && b[1] < 0.99)
          {
            correspondence.a.intensity = a[1];
            correspondence.b.intensity = b[1];
            problem.correspondences.push_back(correspondence);
          }

          if ((channel < 0 || channel == 2) && sa[2] < sb[2] &&
              a[2] > 0.01 && a[2] < 0.99 && b[2] > 0.01 && b[2] < 0.99)
          {
            correspondence.a.intensity = a[2];
            correspondence.b.intensity = b[2];
            problem.correspondences.push_back(correspondence);
          }
        }
      }
    }
  }
}

} // namespace pcalib