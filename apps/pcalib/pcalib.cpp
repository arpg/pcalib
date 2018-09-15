#include <mutex>
#include <thread>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <calibu/target/GridDefinitions.h>
#include <calibu/target/TargetGridDot.h>
#include <calibu/target/RandomGrid.h>
#include <pangolin/pangolin.h>
#include <pcalib/pcalib.h>

DEFINE_string(cam, "", "HAL camera uri");
DEFINE_string(pcalib, "", "input photometric calibration file");
DEFINE_string(output, "pcalib.xml", "output photometric calibration file");
DEFINE_bool(response, true, "enable response calibration");
DEFINE_bool(vignetting, true, "enable vignetting calibration");
DEFINE_double(inlier_thresh, 0.01, "maximum response error for inliers");
DEFINE_int32(ransac_iters, 500, "number of ransac iterations");
DEFINE_int32(keyframe_images, 30, "number of images use for keyframe average");
DEFINE_bool(join_channels, false, "join channels to compute a single response");

DEFINE_int32(grid_height, 0, "");
DEFINE_int32(grid_width, 0, "");
DEFINE_int64(grid_seed, 0, "");
DEFINE_double(grid_large_radius, 0.0, "");
DEFINE_double(grid_small_radius, 0.0, "");
DEFINE_double(grid_spacing, 0.0, "");
DEFINE_string(grid_preset, "", "");

using namespace pcalib;

Image aa;
Image bb;
int eiter = 0;
Eigen::Vector3f emax;
int current_channel;

pangolin::DataLog result_log;
pangolin::View* camera_display;
pangolin::View* initial_display;
pangolin::View* current_display;
std::shared_ptr<pangolin::GlTexture> camera_texture;
std::shared_ptr<pangolin::GlTexture> initial_texture;
std::shared_ptr<pangolin::GlTexture> current_texture;
std::vector<PolynomialResponse> responses;
int last_channel;
PolynomialResponse last_response;
bool response_updated;
std::mutex update_mutex;
bool solved;

inline void ResponseCallback(const PolynomialResponse& response)
{
  std::lock_guard<std::mutex> lock(update_mutex);
  last_response = response;
  response_updated = true;

  if (FLAGS_join_channels)
  {
    responses[0] = response;
    responses[1] = response;
    responses[2] = response;
  }
  else
  {
    responses[current_channel] = response;
  }
}

inline void UpdateResponseTextures()
{
  int lcurrent_channel;
  std::vector<PolynomialResponse> lresponses;
  PolynomialResponse response;

  {
    std::lock_guard<std::mutex> lock(update_mutex);
    if (!response_updated) return;
    response = last_response;
    response_updated = false;
    lcurrent_channel = current_channel;
    lresponses = responses;
  }

  result_log.Clear();

  for (int i = 0; i < 100; ++i)
  {
    const double intensity = double(i) / 99;
    Eigen::Vector3d irradiances(intensity, intensity, intensity);

    for (size_t j = 0; j < responses.size(); ++j)
    {
      irradiances[j] = responses[j](intensity);
    }

    result_log.Log(irradiances[0], irradiances[1], irradiances[2]);
  }

  cv::Mat cc = aa.data().clone();
  cc = cv::Scalar(0);

  const double ae = aa.exposure();
  const double be = bb.exposure();
  const double er = be / ae;

  for (int y = 0; y < cc.rows; ++y)
  {
    for (int x = 0; x < cc.cols; ++x)
    {
      Eigen::Vector3f error(0, 0, 0);
      const Eigen::Vector3f ap = aa.data().at<Eigen::Vector3f>(y, x);
      const Eigen::Vector3f bp = bb.data().at<Eigen::Vector3f>(y, x);
      const bool swap = aa.exposure() > bb.exposure();
      const Eigen::Vector3f sa = swap ? bp : ap;
      const Eigen::Vector3f sb = swap ? ap : bp;

      for (int i = 0; i < 3; ++i)
      {
        if ((lcurrent_channel < 0 || lcurrent_channel == i) && sa[i] < sb[i] &&
            ap[i] > 0.01 && ap[i] < 0.99 && bp[i] > 0.01 && bp[i] < 0.99)
        {
          double ai;
          double bi;

          if (i == lcurrent_channel)
          {
            ai = response(ap[i]);
            bi = response(bp[i]);
          }
          else
          {
            ai = responses[i](ap[i]);
            bi = responses[i](bp[i]);
          }

          const double ir = bi / ai;
          error[i] = std::abs(ir - er);
        }
      }

      cc.at<Eigen::Vector3f>(y, x) = error;
    }
  }

  if (lcurrent_channel != last_channel)
  {
    std::vector<cv::Mat> channels;
    cv::split(cc, channels);

    for (size_t i = 0; i < channels.size(); ++i)
    {
      double cmin, cmax;
      cv::minMaxLoc(channels[i], &cmin, &cmax);
      emax[i] = cmax;

      if (emax[i] > 0) channels[i] /= emax[i];
    }

    cv::Mat dd;
    cv::merge(channels, cc);
    cc.convertTo(dd, CV_8UC3, 255.0);
    const unsigned char* data = dd.data;
    initial_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
    current_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
    last_channel = lcurrent_channel;
  }
  else
  {
    std::vector<cv::Mat> channels;
    cv::split(cc, channels);

    for (size_t i = 0; i < channels.size(); ++i)
    {
      if (emax[i] > 0) channels[i] /= emax[i];
    }

    cv::Mat dd;
    cv::merge(channels, cc);
    cc.convertTo(dd, CV_8UC3, 255.0);
    const unsigned char* data = dd.data;
    current_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
  }
}

std::vector<size_t> sort_indexes(const std::vector<double>& values)
{
  std::vector<size_t> indices(values.size());
  std::iota(indices.begin(), indices.end(), 0);

  std::sort(indices.begin(), indices.end(),
       [&values](size_t i, size_t j) { return values[i] < values[j]; });

  return indices;
}

int main(int argc, char** argv)
{
  // parse arguments

  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing camera uri");
  PCALIB_ASSERT_MSG(!FLAGS_output.empty(), "missing output file");

  LOG(INFO) << "Creating camera...";

  Image image;
  std::shared_ptr<Camera> camera;
  camera = std::make_shared<HalCamera>(FLAGS_cam);
  camera->set_exposure(100);
  camera->set_gain(200);
  camera->Capture(image);
  camera->Capture(image);
  camera->Capture(image);

  std::vector<double> channel_means(3);
  channel_means[0] = image.mean(0);
  channel_means[1] = image.mean(1);
  channel_means[2] = image.mean(2);
  const std::vector<size_t> channel_order = sort_indexes(channel_means);

  std::cout << "means: " <<
      channel_means[0] << " " <<
      channel_means[1] << " " <<
      channel_means[2] << std::endl;

  std::cout << "order: " <<
      channel_order[0] << " " <<
      channel_order[1] << " " <<
      channel_order[2] << std::endl;

  LOG(INFO) << "Creating pangolin displays...";

  pangolin::CreateGlutWindowAndBind("pcalib", 1200, 600);

  camera_display = &pangolin::Display("camera");
  camera_display->SetLock(pangolin::LockCenter, pangolin::LockCenter);
  camera_display->SetBounds(0.0, 1.0, 0.0, 2.0 / 3.0, 640.0 / 480.0);

  pangolin::View& stats_display = pangolin::Display("stats");
  stats_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  stats_display.SetBounds(0.0, 1.0, 2.0 / 3.0, 1.0);

  pangolin::View& upper_stats_display = pangolin::Display("upper stats");
  upper_stats_display.SetLock(pangolin::LockLeft, pangolin::LockTop);
  upper_stats_display.SetBounds(0.75, 1.0, 0.0, 1.0);
  stats_display.AddDisplay(upper_stats_display);

  pangolin::View& lower_stats_display = pangolin::Display("lower stats");
  lower_stats_display.SetLock(pangolin::LockLeft, pangolin::LockBottom);
  lower_stats_display.SetBounds(0.0, 0.75, 0.0, 1.0);
  stats_display.AddDisplay(lower_stats_display);

  initial_display = &pangolin::Display("initial error");
  initial_display->SetLock(pangolin::LockCenter, pangolin::LockCenter);
  initial_display->SetBounds(2.0 / 3.0, 1.0, 0.0, 0.5, 640.0 / 480.0);
  lower_stats_display.AddDisplay(*initial_display);

  current_display = &pangolin::Display("current error");
  current_display->SetLock(pangolin::LockCenter, pangolin::LockCenter);
  current_display->SetBounds(2.0 / 3.0, 1.0, 0.5, 1.0, 640.0 / 480.0);
  lower_stats_display.AddDisplay(*current_display);

  const int w = image.width();
  const int h = image.height();
  camera_texture = std::make_shared<pangolin::GlTexture>(w, h, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  initial_texture = std::make_shared<pangolin::GlTexture>(320, 240, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  current_texture = std::make_shared<pangolin::GlTexture>(320, 240, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

  int target_index = 0;
  std::vector<ExposureTarget> targets;
  const int target_channel = FLAGS_join_channels ? -1 : channel_order[2];

  for (int i = 0; i < 19; ++i)
  {
    targets.push_back(ExposureTarget(50 + 10 * i, target_channel));
  }

  camera->set_exposure_target(targets[target_index]);
  camera->set_exposure(0);
  camera->set_gain(64);

  int frame_count = 0;
  int last_change = 0;
  double last_exposure = 0;

  const int max_fuse_count = FLAGS_keyframe_images;
  bool fusing = false;
  int fuse_count = 0;

  pangolin::DataLog exposure_log;
  std::vector<std::string> exposure_labels;
  exposure_labels.push_back("current intensity (r)");
  exposure_labels.push_back("current intensity (g)");
  exposure_labels.push_back("current intensity (b)");
  exposure_labels.push_back("target intensity");
  exposure_log.SetLabels(exposure_labels);

  const double plot_length = 1.5 * targets.size() * max_fuse_count;
  pangolin::Plotter exposure_plotter(&exposure_log, 0, plot_length, 0, 450);
  pangolin::View& exposure_display = pangolin::Display("exposure_plot");
  exposure_display.SetBounds(0.0, 1.0, 0.0, 1.0);
  exposure_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  exposure_display.AddDisplay(exposure_plotter);
  exposure_plotter.Track("$i");
  upper_stats_display.AddDisplay(exposure_display);

  std::vector<std::string> result_labels;
  result_labels.push_back("response (r)");
  result_labels.push_back("response (g)");
  result_labels.push_back("response (b)");
  result_log.SetLabels(result_labels);

  pangolin::Plotter result_plotter(&result_log, 0, 100, 0, 1.2, 25);
  pangolin::View& result_display = pangolin::Display("result_plot");
  result_display.SetBounds(0.0, 2.0 / 3.0, 0.0, 1.0);
  result_display.SetLock(pangolin::LockLeft, pangolin::LockBottom);
  result_display.AddDisplay(result_plotter);
  lower_stats_display.AddDisplay(result_display);

  for (int i = 0; i < 100; ++i)
  {
    const double intensity = double(i) / 99;
    result_log.Log(intensity, intensity, intensity);
  }

  ResponseProblemBuilder builder;
  builder.set_join_channels(FLAGS_join_channels);
  current_channel = FLAGS_join_channels ? -1 : 0;

  std::vector<Image> images(targets.size());

  while (!pangolin::ShouldQuit())
  {
    camera->Capture(image);

    const double exposure = image.exposure();
    const ExposureTarget& target = targets[target_index];
    const double setpoint = target.intensity;
    exposure_log.Log(image.mean(0), image.mean(1), image.mean(2), setpoint);

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
        cv::imwrite("exposure_" + std::to_string(camera->exposure()) + ".png", mat);

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
      if (frame_count == 0 || std::abs(last_exposure - exposure) / last_exposure > 0.025)
      {
        last_change = frame_count;
      }

      if (frame_count - last_change > 25)
      {
        fusing = true;
        fuse_count = 0;
        camera->set_exposure(exposure);
        LOG(INFO) << "Fusing...";
      }
    }

    glClear(GL_COLOR_BUFFER_BIT);
    camera_display->Activate();
    unsigned char* data = image.data().data;
    camera_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
    camera_texture->RenderToViewportFlipY();

    initial_display->Activate();
    initial_texture->RenderToViewportFlipY();

    current_display->Activate();
    current_texture->RenderToViewportFlipY();

    pangolin::FinishFrame();

    last_exposure = exposure;
    ++frame_count;
  }

  if (!pangolin::ShouldQuit())
  {
    std::shared_ptr<ResponseProblem> problem;
    problem = std::make_shared<ResponseProblem>();
    std::vector<ResponseProblem> problems;
    builder.Build(problems);

    LOG(INFO) << "Created " << problems.size() << " problems";

    std::shared_ptr<PolynomialResponse> response;
    response = std::make_shared<PolynomialResponse>(3);
    responses.resize(3);

    cv::Mat a, b;
    cv::resize(images[1 * targets.size() / 4].data(), a, cv::Size(320, 240), 0, 0, CV_INTER_NN);
    cv::resize(images[3 * targets.size() / 4].data(), b, cv::Size(320, 240), 0, 0, CV_INTER_NN);

    aa = Image(a);
    bb = Image(b);

    aa.set_exposure(images[1 * targets.size() / 4].exposure());
    bb.set_exposure(images[3 * targets.size() / 4].exposure());

    solved = false;
    last_channel = -2;

    for (size_t i = 0; i < responses.size(); ++i) responses[i].set_degree(3);

    std::thread solver_thread([&]()
    {
      for (size_t i = 0; i < problems.size(); ++i)
      {
        {
          std::lock_guard<std::mutex> lock(update_mutex);
          current_channel = FLAGS_join_channels ? -1 : i;
        }

        *response = responses[i];
        ResponseCallback(*response);
        *problem = problems[i];

        LOG(INFO) << "Correspondences: " << problem->correspondences.size();
        ResponseProblemSolver<PolynomialResponse> solver(problem);
        solver.set_inlier_threshold(FLAGS_inlier_thresh);
        solver.set_ransac_iterations(FLAGS_ransac_iters);
        solver.RegisterCallback(ResponseCallback);
        solver.Solve(response);

        {
          std::lock_guard<std::mutex> lock(update_mutex);

          if (FLAGS_join_channels)
          {
            responses[0] = *response;
            responses[1] = *response;
            responses[2] = *response;
          }
          else
          {
            responses[i] = *response;
          }
        }
      }

      solved = true;
    });

    cv::Mat irr_image;
    cv::Mat byte_irr_image;

    while (!pangolin::ShouldQuit())
    {
      camera->Capture(image);
      image.data().convertTo(irr_image, CV_32FC3, 1.0 / 255.0);

      for (int y = 0; y < irr_image.rows; ++y)
      {
        for (int x = 0; x < irr_image.cols; ++x)
        {
          Eigen::Vector3f pixel = irr_image.at<Eigen::Vector3f>(y, x);
          pixel[0] = responses[0](pixel[0]);
          pixel[1] = responses[1](pixel[1]);
          pixel[2] = responses[2](pixel[2]);
          irr_image.at<Eigen::Vector3f>(y, x) = pixel;
        }
      }

      irr_image.convertTo(byte_irr_image, CV_8UC3, 255.0);
      glClear(GL_COLOR_BUFFER_BIT);

      response_updated = true;
      UpdateResponseTextures();

      camera_display->Activate();
      unsigned char* data = byte_irr_image.data;
      camera_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture->RenderToViewportFlipY();

      camera_display->Activate();
      camera_texture->RenderToViewportFlipY();

      initial_display->Activate();
      initial_texture->RenderToViewportFlipY();

      current_display->Activate();
      current_texture->RenderToViewportFlipY();

      pangolin::FinishFrame();

      if (solved)
      {
        solver_thread.join();
        break;
      }
    }

    if (!pangolin::ShouldQuit())
    {
      current_channel = -2;

      std::vector<PolynomialResponse> temp = responses;
      for (PolynomialResponse& response : responses) response.Reset();

      response_updated = true;
      UpdateResponseTextures();

      responses = temp;

      response_updated = true;
      UpdateResponseTextures();

      camera->set_exposure(3 * images[images.size() / 4].exposure());

      Calibration calibration;
      calibration.responses.resize(3);

      for (size_t i = 0; i < calibration.responses.size(); ++i)
      {
        calibration.responses[i] =
            std::make_shared<PolynomialResponse>(responses[i]);
      }

      CalibrationWriter writer(FLAGS_output);
      writer.Write(calibration);
    }

    while (!pangolin::ShouldQuit())
    {
      camera->Capture(image);
      image.data().convertTo(irr_image, CV_32FC3, 1.0 / 255.0);

      for (int y = 0; y < irr_image.rows; ++y)
      {
        for (int x = 0; x < irr_image.cols; ++x)
        {
          Eigen::Vector3f pixel = irr_image.at<Eigen::Vector3f>(y, x);
          pixel[0] = responses[0](pixel[0]);
          pixel[1] = responses[1](pixel[1]);
          pixel[2] = responses[2](pixel[2]);
          irr_image.at<Eigen::Vector3f>(y, x) = pixel;
        }
      }

      irr_image.convertTo(byte_irr_image, CV_8UC3, 255.0);
      glClear(GL_COLOR_BUFFER_BIT);

      camera_display->Activate();
      unsigned char* data = byte_irr_image.data;
      camera_texture->Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture->RenderToViewportFlipY();

      camera_display->Activate();
      camera_texture->RenderToViewportFlipY();

      initial_display->Activate();
      initial_texture->RenderToViewportFlipY();

      current_display->Activate();
      current_texture->RenderToViewportFlipY();

      pangolin::FinishFrame();

      // TODO: vignetting calibration
    }
  }

  LOG(INFO) << "Success";
  return 0;
}