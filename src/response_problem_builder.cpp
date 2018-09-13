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

void ResponseProblemBuilder::AddImage(const Image& image)
{
  PHOTOCALIB_ASSERT_MSG(image.exposure() > 0, "invalid image exposure");

  if (!images_.empty())
  {
    const Image& front = images_.front();
    PHOTOCALIB_ASSERT_MSG(front.width() == image.width(), "size mismatch");
    PHOTOCALIB_ASSERT_MSG(front.height() == image.height(), "size mismatch");
  }

  images_.push_back(image);
}

void ResponseProblemBuilder::Build(ResponseProblem& problem)
{
  PHOTOCALIB_ASSERT_MSG(images_.size() > 1, "insufficient image count");

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

          if (image_i.exposure() < image_j.exposure())
          {
            if (a[0] > b[0] || a[1] > b[1] || a[2] > b[2]) continue;
          }
          else
          {
            if (a[0] < b[0] || a[1] < b[1] || a[2] < b[2]) continue;
          }

          if (a[0] > 0.01 && a[0] < 0.99 && b[0] > 0.01 && b[0] < 0.99)
          {
            correspondence.a.intensity = a[0];
            correspondence.b.intensity = b[0];
            problem.correspondences.push_back(correspondence);
          }

          // if (a[1] > 0.01 && a[1] < 0.99 && b[1] > 0.01 && b[1] < 0.99)
          // {
          //   correspondence.a.intensity = a[1];
          //   correspondence.b.intensity = b[1];
          //   problem.correspondences.push_back(correspondence);
          // }

          // if (a[2] > 0.01 && a[2] < 0.99 && b[2] > 0.01 && b[2] < 0.99)
          // {
          //   correspondence.a.intensity = a[2];
          //   correspondence.b.intensity = b[2];
          //   problem.correspondences.push_back(correspondence);
          // }
        }
      }
    }
  }
}

} // namespace pcalib