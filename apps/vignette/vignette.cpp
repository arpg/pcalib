#include <gflags/gflags.h>
#include <glog/logging.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_xml.h>
#include <calibu/cam/rectify_crtp.h>
#include <calibu/conics/ConicFinder.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/pose/Pnp.h>
#include <calibu/target/GridDefinitions.h>
#include <calibu/target/RandomGrid.h>
#include <calibu/target/TargetGridDot.h>
#include <HAL/Camera/CameraDevice.h>
#include <pangolin/pangolin.h>
#include <pcalib/pcalib.h>

using namespace pcalib;

DEFINE_string(cam, "", "HAL camera uri");
DEFINE_string(calib, "", "camera calibration XML file");
DEFINE_int64(grid_seed, 0, "target grid random seed");
DEFINE_int32(grid_width, 0, "target grid resolution width");
DEFINE_int32(grid_height, 0, "target grid resolution height");
DEFINE_double(grid_spacing, 0.0, "target grid circle spacing");
DEFINE_double(grid_large_radius, 0.0, "target grid large circle radius");
DEFINE_double(grid_small_radius, 0.0, "target grid small circle radius");
DEFINE_string(grid_preset, "", "target grid preset name");
DEFINE_int32(max_samples, 500, "max number of samples per pixel");
DEFINE_int32(max_iters, 3, "max number of optimization steps");

struct Sample
{
  Eigen::Vector3f color;
  Eigen::Vector2i uv;
};

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing HAL camera uri");
  PCALIB_ASSERT_MSG(!FLAGS_calib.empty(), "missing camera calibration");

  typedef Eigen::Matrix<uint8_t, 3, 1> Pixel;

  const double max_color = 255.0;
  std::vector<Pixel> colormap(256);

  for (size_t i = 0; i < colormap.size(); ++i)
  {
    const double t = double(i) / (colormap.size() - 1);
    colormap[i][0] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.75, 2) / 0.15)));
    colormap[i][1] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.50, 2) / 0.15)));
    colormap[i][2] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.25, 2) / 0.15)));
  }

  // create camera

  std::shared_ptr<hal::Camera> raw_camera;
  raw_camera = std::make_shared<hal::Camera>(FLAGS_cam);

  std::shared_ptr<calibu::Rig<double>> rig = calibu::ReadXmlRig(FLAGS_calib);
  PCALIB_ASSERT_MSG(rig->cameras_.size() == 1, "invalid calibration file");
  std::shared_ptr<calibu::CameraInterface<double>> cmod = rig->cameras_[0];

  calibu::LookupTable undistort_table(cmod->Width(), cmod->Height());
  calibu::CreateLookupTable(rig->cameras_[0], undistort_table);

  const int width = raw_camera->Width();
  const int height = raw_camera->Height();
  const int pitch = width * sizeof(unsigned char);
  const double image_aspect_ratio = double(width) / height;

  // create calibu target

  Eigen::MatrixXi grid;
  std::shared_ptr<calibu::TargetGridDot> target;

  int grid_width = 0;
  int grid_height = 0;
  double grid_spacing = 0;
  unsigned int grid_seed = 0;
  double grid_large_radius = 0;
  double grid_small_radius = 0;

  if (FLAGS_grid_preset.empty())
  {
    grid_seed = FLAGS_grid_seed;
    grid_width = FLAGS_grid_width;
    grid_height = FLAGS_grid_height;
    grid_spacing = FLAGS_grid_spacing;
    grid_large_radius = FLAGS_grid_large_radius;
    grid_small_radius = FLAGS_grid_small_radius;

    LOG(INFO) << "Creating custom grid...";
    grid = calibu::MakePattern(grid_height, grid_width, grid_seed);
  }
  else
  {
    const std::string& grid_preset = FLAGS_grid_preset;
    LOG(INFO) << "Creating target from preset: " << grid_preset;

    calibu::LoadGridFromPreset(grid_preset, grid, grid_spacing,
        grid_large_radius, grid_small_radius);
  }

  grid_width = grid.cols();
  grid_height = grid.rows();
  target = std::make_shared<calibu::TargetGridDot>(grid_spacing, grid);

  Eigen::Vector2d target_physical_size;
  target_physical_size[0] = grid_spacing * (grid.cols() - 1);
  target_physical_size[1] = grid_spacing * (grid.rows() - 1);

  // define target reference image dimensions

  int target_reference_width;
  int target_reference_height;
  const double target_aspect_ratio = double(grid.cols()) / grid.rows();

  if (grid.cols() > grid.rows())
  {
    target_reference_width = 640;
    target_reference_height = target_reference_width * target_aspect_ratio;
  }
  else
  {
    target_reference_height = 640;
    target_reference_width = target_reference_height * target_aspect_ratio;
  }

  // build target image mask

  cv::Mat target_mask(target_reference_height, target_reference_width, CV_8UC1);
  target_mask = cv::Scalar(1.0);

  const int conic_count = target->Circles2D().size();

  Eigen::Vector2d target_scaling;
  target_scaling[0] = target_reference_width  / target_physical_size[0];
  target_scaling[1] = target_reference_height / target_physical_size[1];
  const double max_scaling = target_scaling.maxCoeff();

  const Eigen::MatrixXi grid_t = grid.transpose();
  const Eigen::Vector2d grid_radius_sizes(grid_small_radius, grid_large_radius);
  Eigen::Map<const Eigen::VectorXi> grid_radius_map(grid_t.data(), grid.size());

  const double radius_scale = 1.25;

  for (int i = 0; i < conic_count; ++i)
  {
    const Eigen::Vector2d& point = target->Circles2D()[i];
    const Eigen::Vector2d center = point.array() * target_scaling.array();
    const double radius = radius_scale * grid_radius_sizes[grid_radius_map[i]];

    const int x_shift = radius * target_scaling[0];
    const int y_shift = radius * target_scaling[1];

    for (int y = -y_shift; y <= +y_shift; ++y)
    {
      for (int x = -x_shift; x <= +x_shift; ++x)
      {
        const Eigen::Vector2d offset(x, y);
        const Eigen::Vector2d uv = center + offset;

        if (offset.norm() < max_scaling * radius)
        {
          const Eigen::Vector2i pixel = uv.cast<int>();

          if (pixel[0] >= 0 && pixel[0] < target_reference_width &&
              pixel[1] >= 0 && pixel[1] < target_reference_height)
          {
            target_mask.at<unsigned char>(pixel[1], pixel[0]) = 0;
          }
        }
      }
    }
  }

  // create sample buffers

  std::vector<Sample> target_samples(FLAGS_max_samples *
      target_reference_width * target_reference_height);

  std::vector<Sample> image_samples(FLAGS_max_samples * width * height);

  // create pangolin views

  int twidth;
  int theight;

  if (target_reference_width > target_reference_height)
  {
    twidth = target_reference_width;
    theight = target_reference_height;
  }
  else
  {
    twidth = target_reference_height;
    theight = target_reference_width;
  }

  const double reference_aspect_ratio = double(twidth) / theight;
  const double stats_split = 2.0 / 3.0;

  pangolin::CreateGlutWindowAndBind("pcalib", 1080, 600);

  pangolin::View& camera_display = pangolin::Display("camera");
  camera_display.SetLock(pangolin::LockCenter, pangolin::LockCenter);
  camera_display.SetBounds(0.0, 1.0, 0.0, 0.7407, image_aspect_ratio);

  pangolin::View& stats_display = pangolin::Display("stats");
  stats_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  stats_display.SetBounds(0.0, 1.0, 0.7407, 1.0);

  pangolin::View& upper_stats_display = pangolin::Display("upper stats");
  upper_stats_display.SetLock(pangolin::LockLeft, pangolin::LockTop);
  upper_stats_display.SetBounds(stats_split, 1.0, 0.0, 1.0);
  stats_display.AddDisplay(upper_stats_display);

  pangolin::View& lower_stats_display = pangolin::Display("lower stats");
  lower_stats_display.SetLock(pangolin::LockLeft, pangolin::LockBottom);
  lower_stats_display.SetBounds(0.0, stats_split, 0.0, 1.0);
  stats_display.AddDisplay(lower_stats_display);

  pangolin::View& target_display = pangolin::Display("target");
  target_display.SetLock(pangolin::LockCenter, pangolin::LockCenter);
  target_display.SetBounds(0.0, 1.0, 0.0, 1.0, reference_aspect_ratio);
  upper_stats_display.AddDisplay(target_display);

  pangolin::View& vignetting_display = pangolin::Display("vignetting");
  vignetting_display.SetLock(pangolin::LockCenter, pangolin::LockCenter);
  vignetting_display.SetBounds(0.5, 1.0, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(vignetting_display);

  pangolin::View& coverage_display = pangolin::Display("coverage");
  coverage_display.SetLock(pangolin::LockCenter, pangolin::LockCenter);
  coverage_display.SetBounds(0.0, 0.5, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(coverage_display);

  pangolin::GlTexture camera_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture target_texture(twidth, theight, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture vignetting_texture(width, height, GL_RGB, false, 0, GL_RED, GL_UNSIGNED_BYTE);
  pangolin::GlTexture coverage_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

  bool capture_ended = false;
  bool capture_paused = false;

  pangolin::RegisterKeyPressCallback(' ', [&](){ capture_paused = !capture_paused; });
  pangolin::RegisterKeyPressCallback('[', [&](){ capture_ended  = !capture_ended; });
  pangolin::RegisterKeyPressCallback(']', [&](){ capture_ended  = !capture_ended; });

  // capture images

  std::vector<cv::Mat> images;
  raw_camera->Capture(images);
  raw_camera->Capture(images);

  calibu::ImageProcessing processing(width, height);
  calibu::ConicFinder finder;
  std::vector<int> ellipse_map;
  bool target_found;

  cv::Mat target_colors(target_reference_height, target_reference_width, CV_32FC3);
  cv::Mat target_weights(target_reference_height, target_reference_width, CV_16UC1);
  target_colors = cv::Scalar(0, 0, 0);
  target_weights = cv::Scalar(0);

  cv::Mat vignetting_colors(height, width, CV_32FC3);
  cv::Mat vignetting_weights(height, width, CV_16UC1);
  vignetting_colors = cv::Scalar(0, 0, 0);
  vignetting_weights = cv::Scalar(0);

  int frame_number = 0;

  while (!pangolin::ShouldQuit() && !capture_ended)
  {
    if (!capture_paused)
    {
      try
      {
        raw_camera->Capture(images);
      }
      catch (const Exception&)
      {
        capture_ended = true;
        break;
      }

      // TODO: apply inverse response function

      cv::Mat image, float_image;
      PCALIB_ASSERT(images.size() == 1);
      const cv::Mat& cimage = images[0];
      PCALIB_ASSERT(cimage.type() == CV_8UC3);
      cimage.convertTo(float_image, CV_32FC3, 1.0 / 255.0);
      cv::cvtColor(cimage, image, CV_RGB2GRAY);

      processing.Process(image.data, width, height, pitch);
      finder.Find(processing);
      target_found = target->FindTarget(processing, finder.Conics(), ellipse_map);

      if (target_found)
      {
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> ellipses(finder.Conics().size());

        for (size_t i = 0; i < finder.Conics().size(); ++i)
        {
          ellipses[i] = finder.Conics()[i].center;
        }

        Sophus::SE3d Tcw;
        const int ransac_iterations = 0;
        const float ransac_tolerance = 0.0f;

        // compute current world-to-camera pose as per rectified image

        calibu::PosePnPRansac(cmod, ellipses, target->Circles3D(), ellipse_map,
            ransac_iterations, ransac_tolerance, &Tcw);

        const double error = calibu::ReprojectionErrorRMS(rig->cameras_[0], Tcw,
            target->Circles3D(), ellipses, ellipse_map);

        if (error < 2.0)
        {
          const Sophus::SE3d Twc = Tcw.inverse();

          for (int y = 0; y < height; ++y)
          {
            for (int x = 0; x < width; ++x)
            {
              const Eigen::Vector2d uv(x + 0.5, y + 0.5);
              const Eigen::Vector3d Xcp = cmod->Unproject(uv);
              const Eigen::Vector3d origin = Twc.translation();
              const Eigen::Vector3d Xwp = Twc * Xcp;
              const Eigen::Vector3d dir = Xwp - origin;

              if (std::abs(dir[2]) < 1E-8) continue;

              const double t = -origin[2] / dir[2];
              const Eigen::Vector3d p = origin + t * dir;

              if (p[0] < 0 || p[0] >= target_physical_size[0] ||
                  p[1] < 0 || p[1] >= target_physical_size[1])
              {
                Pixel pixel = images[0].at<Pixel>(y, x);
                pixel[0] = std::min(255, int(pixel[0]) + 32);
                images[0].at<Pixel>(y, x) = pixel;
                continue;
              }

              const int tx = p[0] * target_scaling[0];
              const int ty = p[1] * target_scaling[1];
              const bool mask = target_mask.at<unsigned char>(ty, tx);

              if (!mask)
              {
                Pixel pixel = images[0].at<Pixel>(y, x);
                pixel[0] = std::min(255, int(pixel[0]) + 32);
                images[0].at<Pixel>(y, x) = pixel;
                continue;
              }

              const Eigen::Vector3f color = float_image.at<Eigen::Vector3f>(y, x);

              if (color[0] < 0.01 || color[0] > 0.99 ||
                  color[0] < 0.01 || color[0] > 0.99 ||
                  color[0] < 0.01 || color[0] > 0.99)
              {
                continue;
              }

              const Eigen::Vector3f old_color = target_colors.at<Eigen::Vector3f>(ty, tx);
              const Eigen::Vector3f old_vcolor = vignetting_colors.at<Eigen::Vector3f>(y, x);

              const int old_weight = target_weights.at<uint16_t>(ty, tx);
              const int new_weight = old_weight + 1;

              Eigen::Vector3f tcolor = color;

              if (frame_number > 0)
              {
                tcolor[0] *= old_vcolor[0];
                tcolor[1] *= old_vcolor[1];
                tcolor[2] *= old_vcolor[2];
              }

              const Eigen::Vector3f new_color = (old_weight * old_color + tcolor) / new_weight;

              target_weights.at<uint16_t>(ty, tx) = std::min(FLAGS_max_samples, new_weight);
              target_colors.at<Eigen::Vector3f>(ty, tx) = new_color;

              Eigen::Vector3f vcolor(1, 1, 1);

              if (frame_number > 0)
              {
                vcolor[0] = old_color[0] / color[0];
                vcolor[1] = old_color[1] / color[1];
                vcolor[2] = old_color[2] / color[2];
              }

              const int old_vweight = vignetting_weights.at<uint16_t>(y, x);
              const int new_vweight = old_vweight + 1;

              const Eigen::Vector3f new_vcolor = (old_vweight * old_vcolor + vcolor) / new_vweight;

              vignetting_weights.at<uint16_t>(y, x) = std::min(FLAGS_max_samples, new_vweight);
              vignetting_colors.at<Eigen::Vector3f>(y, x) = new_vcolor;

              if (new_weight <= FLAGS_max_samples)
              {
                Sample sample;
                sample.color = color;
                sample.uv = Eigen::Vector2i(x, y);

                const int index =
                    ty * target_reference_width * FLAGS_max_samples +
                    tx * FLAGS_max_samples +
                    new_weight - 1;

                target_samples[index] = sample;
              }

              if (new_vweight <= FLAGS_max_samples)
              {
                Sample sample;
                sample.color = color;
                sample.uv = Eigen::Vector2i(tx, ty);

                const int index =
                    y * width * FLAGS_max_samples +
                    x * FLAGS_max_samples +
                    new_vweight - 1;

                image_samples[index] = sample;
              }
            }
          }
        }
      }

      ++frame_number;
    }

    glClear(GL_COLOR_BUFFER_BIT);

    {
      camera_display.Activate();
      unsigned char* data = images[0].data;
      camera_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      target_colors.convertTo(tmp, CV_8UC3, 255.0);

      if (target_reference_width < target_reference_height)
      {
        tmp = tmp.t();
      }

      target_display.Activate();
      unsigned char* data = tmp.data;
      target_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      target_texture.RenderToViewport();
    }

    {
      cv::Mat tmp;
      vignetting_colors.convertTo(tmp, CV_8UC3, 255.0);
      vignetting_display.Activate();
      unsigned char* data = tmp.data;
      vignetting_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      vignetting_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      vignetting_weights.convertTo(tmp, CV_8UC1, 255.0 / FLAGS_max_samples);
      cv::cvtColor(tmp, tmp, CV_GRAY2BGR);

      for (int y = 0; y < tmp.rows; ++y)
      {
        for (int x = 0; x < tmp.cols; ++x)
        {
          tmp.at<Pixel>(y, x) = colormap[tmp.at<Pixel>(y, x)[0]];
        }
      }

      coverage_display.Activate();
      unsigned char* data = tmp.data;
      coverage_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      coverage_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }

  for (int i = 0; i < FLAGS_max_iters && !pangolin::ShouldQuit(); ++i)
  {
    // update scene irradiance

    LOG(INFO) << "Updating scene irradiance...";

    for (int y = 0; y < target_reference_height; ++y)
    {
      for (int x = 0; x < target_reference_width; ++x)
      {
        const int count = target_weights.at<uint16_t>(y, x);

        if (count == 0) continue;

        std::vector<float> r(count);
        std::vector<float> g(count);
        std::vector<float> b(count);

        for (int s = 0; s < count; ++s)
        {
          const int index =
              y * target_reference_width * FLAGS_max_samples +
              x * FLAGS_max_samples +
              s;

          const Sample& sample = target_samples[index];
          const int ix = sample.uv[0];
          const int iy = sample.uv[1];

          const Eigen::Vector3f vig = vignetting_colors.at<Eigen::Vector3f>(iy, ix);

          r[s] = sample.color[0] * vig[0];
          g[s] = sample.color[1] * vig[1];
          b[s] = sample.color[2] * vig[2];
        }

        std::sort(r.begin(), r.end());
        std::sort(g.begin(), g.end());
        std::sort(b.begin(), b.end());

        Eigen::Vector3f color;
        color[0] = 0.5 * (r[(count - 1) / 2] + r[count / 2]);
        color[1] = 0.5 * (g[(count - 1) / 2] + g[count / 2]);
        color[2] = 0.5 * (b[(count - 1) / 2] + b[count / 2]);
        target_colors.at<Eigen::Vector3f>(y, x) = color;
      }
    }

    glClear(GL_COLOR_BUFFER_BIT);

    {
      camera_display.Activate();
      unsigned char* data = images[0].data;
      camera_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      target_colors.convertTo(tmp, CV_8UC3, 255.0);

      if (target_reference_width < target_reference_height)
      {
        tmp = tmp.t();
      }

      target_display.Activate();
      unsigned char* data = tmp.data;
      target_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      target_texture.RenderToViewport();
    }

    {
      cv::Mat tmp;
      vignetting_colors.convertTo(tmp, CV_8UC3, 255.0);
      vignetting_display.Activate();
      unsigned char* data = tmp.data;
      vignetting_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      vignetting_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      vignetting_weights.convertTo(tmp, CV_8UC1, 255.0 / FLAGS_max_samples);
      cv::cvtColor(tmp, tmp, CV_GRAY2BGR);

      for (int y = 0; y < tmp.rows; ++y)
      {
        for (int x = 0; x < tmp.cols; ++x)
        {
          tmp.at<Pixel>(y, x) = colormap[tmp.at<Pixel>(y, x)[0]];
        }
      }

      coverage_display.Activate();
      unsigned char* data = tmp.data;
      coverage_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      coverage_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();

    // update vignetting

    LOG(INFO) << "Updating vignetting...";

    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        const int count = vignetting_weights.at<uint16_t>(y, x);

        if (count == 0) continue;

        std::vector<float> r(count);
        std::vector<float> g(count);
        std::vector<float> b(count);

        for (int s = 0; s < count; ++s)
        {
          const int index =
              y * width * FLAGS_max_samples +
              x * FLAGS_max_samples +
              s;

          const Sample& sample = image_samples[index];
          const int tx = sample.uv[0];
          const int ty = sample.uv[1];

          const Eigen::Vector3f irr = target_colors.at<Eigen::Vector3f>(ty, tx);

          r[s] = irr[0] / sample.color[0];
          g[s] = irr[1] / sample.color[1];
          b[s] = irr[2] / sample.color[2];
        }

        std::sort(r.begin(), r.end());
        std::sort(g.begin(), g.end());
        std::sort(b.begin(), b.end());

        Eigen::Vector3f color;
        color[0] = 0.5 * (r[(count - 1) / 2] + r[count / 2]);
        color[1] = 0.5 * (g[(count - 1) / 2] + g[count / 2]);
        color[2] = 0.5 * (b[(count - 1) / 2] + b[count / 2]);
        vignetting_colors.at<Eigen::Vector3f>(y, x) = color;
      }
    }

    glClear(GL_COLOR_BUFFER_BIT);

    {
      camera_display.Activate();
      unsigned char* data = images[0].data;
      camera_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      target_colors.convertTo(tmp, CV_8UC3, 255.0);

      if (target_reference_width < target_reference_height)
      {
        tmp = tmp.t();
      }

      target_display.Activate();
      unsigned char* data = tmp.data;
      target_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      target_texture.RenderToViewport();
    }

    {
      cv::Mat tmp;
      vignetting_colors.convertTo(tmp, CV_8UC3, 255.0);
      vignetting_display.Activate();
      unsigned char* data = tmp.data;
      vignetting_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      vignetting_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      vignetting_weights.convertTo(tmp, CV_8UC1, 255.0 / FLAGS_max_samples);
      cv::cvtColor(tmp, tmp, CV_GRAY2BGR);

      for (int y = 0; y < tmp.rows; ++y)
      {
        for (int x = 0; x < tmp.cols; ++x)
        {
          tmp.at<Pixel>(y, x) = colormap[tmp.at<Pixel>(y, x)[0]];
        }
      }

      coverage_display.Activate();
      unsigned char* data = tmp.data;
      coverage_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      coverage_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }

  while (!pangolin::ShouldQuit())
  {
    if (!capture_paused)
    {
      try
      {
        raw_camera->Capture(images);
      }
      catch (const Exception&)
      {
        capture_ended = true;
        break;
      }

      // TODO: apply inverse response function
      // TODO: remove vignetting
    }

    glClear(GL_COLOR_BUFFER_BIT);

    {
      camera_display.Activate();
      unsigned char* data = images[0].data;
      camera_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      target_colors.convertTo(tmp, CV_8UC3, 255.0);

      if (target_reference_width < target_reference_height)
      {
        tmp = tmp.t();
      }

      target_display.Activate();
      unsigned char* data = tmp.data;
      target_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      target_texture.RenderToViewport();
    }

    {
      cv::Mat tmp;
      vignetting_colors.convertTo(tmp, CV_8UC3, 255.0);
      vignetting_display.Activate();
      unsigned char* data = tmp.data;
      vignetting_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      vignetting_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat tmp;
      vignetting_weights.convertTo(tmp, CV_8UC1, 255.0 / FLAGS_max_samples);
      cv::cvtColor(tmp, tmp, CV_GRAY2BGR);

      for (int y = 0; y < tmp.rows; ++y)
      {
        for (int x = 0; x < tmp.cols; ++x)
        {
          tmp.at<Pixel>(y, x) = colormap[tmp.at<Pixel>(y, x)[0]];
        }
      }

      coverage_display.Activate();
      unsigned char* data = tmp.data;
      coverage_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      coverage_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }

  LOG(INFO) << "Success";
  return 0;
}