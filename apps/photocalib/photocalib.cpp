#include <gflags/gflags.h>
#include <glog/logging.h>
#include <calibu/target/GridDefinitions.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/target/RandomGrid.h>
#include <pangolin/pangolin.h>
#include <photocalib/photocalib.h>

DEFINE_string(cam, "", "HAL camera uri");
DEFINE_string(pcalib, "", "input photometric calibration file");
DEFINE_string(output, "pcalib.xml", "output photometric calibration file");
DEFINE_bool(response, true, "enable response calibration");
DEFINE_bool(vignetting, true, "enable vignetting calibration");
DEFINE_double(inlier_thresh, 0.1, "maximum response error for inliers");
DEFINE_int32(ransac_iters, 1000, "number of ransac iterations");

DEFINE_int32(grid_height, 0, "");
DEFINE_int32(grid_width, 0, "");
DEFINE_int64(grid_seed, 0, "");
DEFINE_double(grid_large_radius, 0.0, "");
DEFINE_double(grid_small_radius, 0.0, "");
DEFINE_double(grid_spacing, 0.0, "");
DEFINE_string(grid_preset, "", "");

using namespace photocalib;

int main(int argc, char** argv)
{
  // parse arguments

  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PHOTOCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing camera uri");
  PHOTOCALIB_ASSERT_MSG(!FLAGS_output.empty(), "missing output file");

  LOG(INFO) << "Creating camera...";

  Image image;
  std::shared_ptr<Camera> camera;
  camera = std::make_shared<HalCamera>(FLAGS_cam);
  camera->set_exposure(50);
  camera->set_gain(1);
  camera->Capture(image);

  pangolin::CreateGlutWindowAndBind("Photocalib", 1400, 600);

  pangolin::View& display = pangolin::Display("camera");
  display.SetLock(pangolin::LockLeft, pangolin::LockTop);
  display.SetBounds(0.0, 1.0, 0.0, 2.0 / 3.0, 640.0 / 480.0);

  const int w = image.width();
  const int h = image.height();
  pangolin::GlTexture texture(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

  int target_index = 0;
  std::vector<ExposureTarget> targets;
  targets.push_back(ExposureTarget( 10));
  targets.push_back(ExposureTarget( 25));
  targets.push_back(ExposureTarget( 50));
  targets.push_back(ExposureTarget(100));
  targets.push_back(ExposureTarget(150));
  targets.push_back(ExposureTarget(200));
  targets.push_back(ExposureTarget(225));
  targets.push_back(ExposureTarget(245));
  camera->set_exposure_target(targets[target_index]);
  camera->set_exposure(0);
  camera->set_gain(1);

  int frame_count = 0;
  int last_change = 0;
  double last_exposure = 0;

  const int max_fuse_count = 75;
  bool fusing = false;
  int fuse_count = 0;

  pangolin::DataLog exposure_log;
  std::vector<std::string> exposure_labels;
  exposure_labels.push_back("target intensity");
  exposure_labels.push_back("current intensity");
  exposure_log.SetLabels(exposure_labels);

  const double plot_length = 1.5 * targets.size() * max_fuse_count;
  pangolin::Plotter exposure_plotter(&exposure_log, 0, plot_length, 0, 350);
  pangolin::View& exposure_display = pangolin::Display("exposure_plot");
  exposure_display.SetBounds(0.25, 1.0, 0.6, 1.0, 4.0);
  exposure_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  exposure_display.AddDisplay(exposure_plotter);
  exposure_plotter.Track("$i");

  pangolin::DataLog result_log;
  std::vector<std::string> result_labels;
  result_labels.push_back("response");
  result_log.SetLabels(result_labels);

  pangolin::Plotter result_plotter(&result_log, 0, 100, 0, 1.1, 25);
  pangolin::View& result_display = pangolin::Display("result_plot");
  result_display.SetBounds(0.0, 0.4, 0.6, 1.0, 0.91);
  result_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  result_display.AddDisplay(result_plotter);

  for (int i = 0; i < 100; ++i)
  {
    const double intensity = double(i) / 99;
    result_log.Log(intensity);
  }

  ResponseProblemBuilder builder;

  std::vector<Image> images(targets.size());

  while (!pangolin::ShouldQuit())
  {
    camera->Capture(image);

    const double exposure = image.exposure();
    const ExposureTarget& target = targets[target_index];
    const double feedback = image.mean(target.channel);
    const double setpoint = target.intensity;
    exposure_log.Log(setpoint, feedback);

    if (fusing)
    {
      if (fuse_count == 0)
      {
        cv::Mat data = image.data().clone();

        switch (data.type())
        {
          case CV_8UC1:
            data.convertTo(data, CV_32FC1, 1.0 / 255.0);
            break;

          case CV_8UC3:
            data.convertTo(data, CV_32FC3, 1.0 / 255.0);
            break;

          case CV_16UC1:
            data.convertTo(data, CV_32FC1, 1.0 / 65535.0);
            break;

          case CV_16UC3:
            data.convertTo(data, CV_32FC3, 1.0 / 65535.0);
            break;
        }

        Image imagef(data);
        imagef.set_exposure(image.exposure());
        images[target_index] = imagef;
      }
      else
      {
        cv::Mat data = image.data().clone();

        switch (data.type())
        {
          case CV_8UC1:
            data.convertTo(data, CV_32FC1, 1.0 / 255.0);
            break;

          case CV_8UC3:
            data.convertTo(data, CV_32FC3, 1.0 / 255.0);
            break;

          case CV_16UC1:
            data.convertTo(data, CV_32FC1, 1.0 / 65535.0);
            break;

          case CV_16UC3:
            data.convertTo(data, CV_32FC3, 1.0 / 65535.0);
            break;
        }

        Image imagef(data);
        imagef.set_exposure(image.exposure());
        images[target_index] += imagef;
      }

      ++fuse_count;

      if (fuse_count == max_fuse_count)
      {
        images[target_index] *= 1.0 / max_fuse_count;
        builder.AddImage(images[target_index]);

        cv::Mat mat = images[target_index].data().clone();
        mat.convertTo(mat, CV_8UC3, 255.0);
        cv::cvtColor(mat, mat, CV_BGR2RGB);
        cv::imwrite("image" + std::to_string(target_index) + ".png", mat);

        target_index = (target_index + 1) % targets.size();

        if (target_index == 0)
        {
          break;
        }

        last_change = frame_count;
        camera->set_exposure_target(targets[target_index]);
        camera->set_exposure(0);
        fusing = false;
      }
    }
    else
    {
      if (frame_count == 0 || std::abs(last_exposure - exposure) > 0.1)
      {
        last_change = frame_count;
      }

      if (frame_count - last_change > 15)
      {
        fusing = true;
        fuse_count = 0;
        camera->set_exposure(exposure);
        LOG(INFO) << "Fusing...";
      }
    }

    glClear(GL_COLOR_BUFFER_BIT);
    display.Activate();
    unsigned char* data = image.data().data;
    texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
    texture.RenderToViewportFlipY();
    pangolin::FinishFrame();

    last_exposure = exposure;
    ++frame_count;
  }

  if (!pangolin::ShouldQuit())
  {
    std::shared_ptr<ResponseProblem> problem;
    problem = std::make_shared<ResponseProblem>();
    builder.Build(*problem);
    LOG(INFO) << "Correspondences: " << problem->correspondences.size();

    std::shared_ptr<PolynomialResponse> response;
    response = std::make_shared<PolynomialResponse>(3);

    ResponseProblemSolver<PolynomialResponse> solver(problem);
    solver.set_inlier_threshold(FLAGS_inlier_thresh);
    solver.set_ransac_iterations(FLAGS_ransac_iters);
    solver.Solve(response);

    Eigen::VectorXd params = response->parameters();
    LOG(INFO) << "unnormalized params: " << params.transpose();
    params /= (*response)(1.0);
    LOG(INFO) << "normalized params  : " << params.transpose();
    response->set_parameters(params);

    result_log.Clear();

    for (int i = 0; i < 100; ++i)
    {
      const double intensity = double(i) / 99;
      const double irradiance = (*response)(intensity);
      result_log.Log(irradiance);
    }

    while (!pangolin::ShouldQuit())
    {
      glClear(GL_COLOR_BUFFER_BIT);
      display.Activate();
      texture.RenderToViewportFlipY();
      pangolin::FinishFrame();

      // TODO: vignetting calibration
    }
  }

  LOG(INFO) << "Success";
  return 0;
}