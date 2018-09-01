#include <gflags/gflags.h>
#include <glog/logging.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/rectify_crtp.h>
#include <calibu/cam/camera_xml.h>
#include <calibu/conics/ConicFinder.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/pose/Pnp.h>
#include <calibu/target/GridDefinitions.h>
#include <calibu/target/RandomGrid.h>
#include <calibu/target/TargetGridDot.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/Camera/Drivers/Undistort/UndistortDriver.h>
#include <pangolin/pangolin.h>
#include <photocalib/photocalib.h>

DEFINE_string(cam, "", "HAL camera uri");
DEFINE_string(calib, "", "camera calibration file");
DEFINE_int64(grid_seed, 0, "target grid random seed");
DEFINE_int32(grid_width, 0, "target grid resolution width");
DEFINE_int32(grid_height, 0, "target grid resolution height");
DEFINE_double(grid_spacing, 0.0, "target grid circle spacing");
DEFINE_double(grid_large_radius, 0.0, "target grid large circle radius");
DEFINE_double(grid_small_radius, 0.0, "target grid small circle radius");
DEFINE_string(grid_preset, "", "target grid preset name");
DEFINE_string(output_image, "vignetting.png", "vignetting image output file");

using namespace photocalib;

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PHOTOCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing HAL camera uri");
  PHOTOCALIB_ASSERT_MSG(!FLAGS_calib.empty(), "missing camera calibration");

  typedef Eigen::Matrix<unsigned char, 3, 1> Pixel;

  const double max_color = 255.0;
  std::vector<Pixel> colormap(256);

  for (size_t i = 0; i < colormap.size(); ++i)
  {
    const double t = double(i) / (colormap.size() - 1);
    colormap[i][0] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.75, 2) / 0.15)));
    colormap[i][1] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.50, 2) / 0.15)));
    colormap[i][2] = std::max(0.0, std::min(max_color, 2.1 * max_color - max_color * std::exp(std::pow(t - 0.25, 2) / 0.15)));
  }

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

  Eigen::Vector2d target_physical_size;
  target_physical_size[0] = grid_spacing * (grid.cols() - 1);
  target_physical_size[1] = grid_spacing * (grid.rows() - 1);

  Eigen::Vector2d target_scaling;
  target_scaling[0] = target_reference_width  / target_physical_size[0];
  target_scaling[1] = target_reference_height / target_physical_size[1];
  const double max_scaling = target_scaling.maxCoeff();

  const Eigen::MatrixXi grid_t = grid.transpose();
  const Eigen::Vector2d grid_radius_sizes(grid_small_radius, grid_large_radius);
  Eigen::Map<const Eigen::VectorXi> grid_radius_map(grid_t.data(), grid.size());

  const double radius_scale = 1.35;

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

  std::shared_ptr<calibu::Rig<double>> rig;
  rig = calibu::ReadXmlRig(FLAGS_calib);

  std::shared_ptr<hal::Camera> raw_camera;
  raw_camera = std::make_shared<hal::Camera>(FLAGS_cam);

  // programmatrically create new pcalib camera interface
    // myself for now, use hal later once integrated

  PolynomialResponse response(3);

  // // orbbec
  // response.set_parameters(Eigen::Vector3d(0.999711, -1.37383, 1.37412));

  // xtion
  response.set_parameters(Eigen::Vector3d(0.506497, .0934983, 0.400005));

  const int width = raw_camera->Width();
  const int height = raw_camera->Height();
  const int pitch = width * sizeof(unsigned char);
  const double image_aspect_ratio = double(width) / height;

  calibu::LookupTable undistort_lookup(width, height);
  calibu::CreateLookupTable(rig->cameras_[0], undistort_lookup);

  std::vector<cv::Mat> images;
  raw_camera->Capture(images);

  cv::Mat raw_image(height, width, CV_8UC1);
  cv::Mat undistort_image(height, width, CV_8UC1);

  calibu::ConicFinder finder;
  calibu::ImageProcessing processing(width, height);
  std::vector<int> ellipse_map;
  bool target_found;

  cv::Mat reference_image(target_reference_height, target_reference_width, CV_32FC1);

  cv::Mat vignetting(height, width, CV_32FC1);
  cv::Mat weights(height, width, CV_32FC1);

  vignetting = cv::Scalar(1);
  weights = cv::Scalar(0);

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
  const double stats_split = 1.0;

  pangolin::CreateGlutWindowAndBind("Photocalib", 1200, 600);

  pangolin::View& camera_display = pangolin::Display("camera");
  camera_display.SetLock(pangolin::LockCenter, pangolin::LockCenter);
  camera_display.SetBounds(0.0, 1.0, 0.0, 2.0 / 3.0, image_aspect_ratio);

  pangolin::View& stats_display = pangolin::Display("stats");
  stats_display.SetLock(pangolin::LockRight, pangolin::LockTop);
  stats_display.SetBounds(0.0, 1.0, 2.0 / 3.0, 1.0);

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
  vignetting_display.SetLock(pangolin::LockLeft, pangolin::LockTop);
  vignetting_display.SetBounds(0.5, 1.0, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(vignetting_display);

  pangolin::View& coverage_display = pangolin::Display("coverage");
  coverage_display.SetLock(pangolin::LockRight, pangolin::LockBottom);
  coverage_display.SetBounds(0.0, 0.5, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(coverage_display);

  pangolin::GlTexture camera_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture target_texture(twidth, theight, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture vignetting_texture(width, height, GL_RGB, false, 0, GL_RED, GL_UNSIGNED_BYTE);
  pangolin::GlTexture coverage_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

  cv::Mat vig_values(height, width, CV_32FC1);
  cv::Mat vig_weights(height, width, CV_32FC1);

  vig_values = cv::Scalar(0);
  vig_weights = cv::Scalar(0);

  bool capture_ended = false;
  bool capture_paused = false;
  pangolin::RegisterKeyPressCallback(' ', [&](){ capture_paused = !capture_paused; });
  pangolin::RegisterKeyPressCallback('[', [&](){ capture_ended  = !capture_ended; });
  pangolin::RegisterKeyPressCallback(']', [&](){ capture_ended  = !capture_ended; });

  cv::Mat new_mask = target_mask.clone();

  while (!pangolin::ShouldQuit() && !capture_ended)
  {
    glClear(GL_COLOR_BUFFER_BIT);

    if (!capture_paused)
    {
      // capture raw image

      try
      {
        raw_camera->Capture(images);
      }
      catch (const Exception& exception)
      {
        capture_ended = true;
        break;
      }

      // convert image to grayscale

      cv::cvtColor(images[0], raw_image, CV_BGR2GRAY);

      // undistort raw image

      unsigned char* dst = undistort_image.data;
      const unsigned char* src = raw_image.data;
      calibu::Rectify<unsigned char>(undistort_lookup, src, dst, width, height);

      // process rectified image for conic extraction

      processing.Process(undistort_image.data, width, height, pitch);

      // find conics in rectified image

      finder.Find(processing);

      // detect target from found conics

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

        calibu::PosePnPRansac(rig->cameras_[0], ellipses, target->Circles3D(),
            ellipse_map, ransac_iterations, ransac_tolerance, &Tcw);

        const double error = calibu::ReprojectionErrorRMS(rig->cameras_[0], Tcw,
            target->Circles3D(), ellipses, ellipse_map);

        if (error < 2.0)
        {
          Eigen::Vector2d target_scaling;
          target_scaling[0] = target_reference_width  / target_physical_size[0];
          target_scaling[1] = target_reference_height / target_physical_size[1];

          reference_image = cv::Scalar(0);

          new_mask = target_mask.clone();

          for (int y = 0; y < images[0].rows; ++y)
          {
            for (int x = 0; x < images[0].cols; ++x)
            {
              const Pixel& pixel = images[0].at<Pixel>(y, x);
              const Eigen::Vector3f intensity = pixel.cast<float>() / 255;

              Eigen::Vector3f irradiance;
              irradiance[0] = response(intensity[0]);
              irradiance[1] = response(intensity[1]);
              irradiance[2] = response(intensity[3]);

              const Eigen::Vector2d uv = Eigen::Vector2d(x + 0.5, y + 0.5);
              const Eigen::Vector3d Xcp = rig->cameras_[0]->Unproject(uv);
              const Eigen::Vector3d Xwp = Tcw.inverse() * Xcp;
              const Eigen::Vector3d origin = Tcw.inverse().translation();
              const Eigen::Vector3d dir = Xwp - origin;

              if (std::abs(dir[2]) < 1E-8)
              {
                Pixel new_pixel = pixel;
                new_pixel[0] = std::min(255, new_pixel[0] + 75);
                images[0].at<Pixel>(y, x) = new_pixel;
                continue;
              }

              const double t = -origin[2] / dir[2];
              const Eigen::Vector3d p = origin + t * dir;

              if (p[0] < 0 || p[0] > target_physical_size[0] ||
                  p[1] < 0 || p[1] > target_physical_size[1])
              {
                Pixel new_pixel = pixel;
                new_pixel[0] = std::min(255, new_pixel[0] + 75);
                images[0].at<Pixel>(y, x) = new_pixel;
                continue;
              }

              const int tx = p[0] * target_scaling[0];
              const int ty = p[1] * target_scaling[1];
              const bool mask = target_mask.at<unsigned char>(ty, tx);

              if (!mask)
              {
                Pixel new_pixel = pixel;
                new_pixel[0] = std::min(255, new_pixel[0] + 75);
                images[0].at<Pixel>(y, x) = new_pixel;
                continue;
              }

              float value = 0.0;
              int count = 0;

              if (pixel[0] > 2 && pixel[0] < 253)
              {
                value += irradiance[0];
                ++count;
              }

              if (pixel[1] > 2 && pixel[1] < 253)
              {
                value += irradiance[1];
                ++count;
              }

              if (pixel[2] > 2 && pixel[2] < 253)
              {
                value += irradiance[2];
                ++count;
              }

              if (count > 0)
              {
                value /= count;

                const float old_value = vig_values.at<float>(y, x);
                const float old_weight = vig_weights.at<float>(y, x);

                const float new_weight = old_weight + 1;
                const float new_value = (old_weight * old_value + value) / new_weight;


                vig_values.at<float>(y, x) = new_value;
                vig_weights.at<float>(y, x) = new_weight;
              }
            }
          }


          // for each pixel in the target reference image

          for (int y = 0; y < reference_image.rows; ++y)
          {
            for (int x = 0; x < reference_image.cols; ++x)
            {
              const Eigen::Vector2d reference_uv(x + 0.5, y + 0.5);

              // reference space -> target space
              double ru = reference_uv[0] / target_scaling[0];
              double rv = reference_uv[1] / target_scaling[1];

              // target space -> grid space
              ru = ru / grid_spacing;
              rv = rv / grid_spacing;

              // get grid corner indices
              const int ui = ru; // TODO: handle border cases
              const int vi = rv;

              // get 4 grid weights
              const double wu1 = ru - ui;
              const double wv1 = rv - vi;
              const double wu0 = 1 - wu1;
              const double wv0 = 1 - wv1;

              // get 4 grid indices
              const int i00 = (vi + 0) * grid_width + (ui + 0);
              const int i01 = (vi + 0) * grid_width + (ui + 1);
              const int i10 = (vi + 1) * grid_width + (ui + 0);
              const int i11 = (vi + 1) * grid_width + (ui + 1);

              // get 4 grid 3D points
              const Eigen::Vector3d& Xwp00 = target->Circles3D()[i00];
              const Eigen::Vector3d& Xwp01 = target->Circles3D()[i01];
              const Eigen::Vector3d& Xwp10 = target->Circles3D()[i10];
              const Eigen::Vector3d& Xwp11 = target->Circles3D()[i11];

              // bilinear interp 3D points
              Eigen::Vector3d Xwp(0, 0, 0);
              Xwp += wv0 * wu0 * Xwp00;
              Xwp += wv0 * wu1 * Xwp01;
              Xwp += wv1 * wu0 * Xwp10;
              Xwp += wv1 * wu1 * Xwp11;

              const Eigen::Vector3d Xcp = Tcw * Xwp;
              const Eigen::Vector2d undistort_uv = rig->cameras_[0]->Project(Xcp);

              if (undistort_uv[0] >= 0.5 && undistort_uv[0] < undistort_image.cols - 0.5 &&
                  undistort_uv[1] >= 0.5 && undistort_uv[1] < undistort_image.rows - 0.5)
              {
                const Eigen::Vector2i pixel = (undistort_uv - Eigen::Vector2d(0.5, 0.5)).cast<int>();
                const Eigen::Vector2d w1 = undistort_uv - (pixel.cast<double>() + Eigen::Vector2d(0.5, 0.5));
                const Eigen::Vector2d w0 = Eigen::Vector2d(1, 1) - w1;

                const unsigned char int00 = undistort_image.at<unsigned char>(pixel[1] + 0, pixel[0] + 0);
                const unsigned char int01 = undistort_image.at<unsigned char>(pixel[1] + 0, pixel[0] + 1);
                const unsigned char int10 = undistort_image.at<unsigned char>(pixel[1] + 1, pixel[0] + 0);
                const unsigned char int11 = undistort_image.at<unsigned char>(pixel[1] + 1, pixel[0] + 1);

                const float irr00 = response(int00 / 255.0);
                const float irr01 = response(int01 / 255.0);
                const float irr10 = response(int10 / 255.0);
                const float irr11 = response(int11 / 255.0);

                double irradiance = 0;
                irradiance += w0[1] * w0[0] * irr00;
                irradiance += w0[1] * w1[0] * irr01;
                irradiance += w1[1] * w0[0] * irr10;
                irradiance += w1[1] * w1[0] * irr11;
                reference_image.at<float>(y, x) = irradiance;
              }
              else
              {
                new_mask.at<unsigned char>(y, x) = 0;
              }
            }
          }
        }
      }
    }

    cv::Mat fmask = 1 - new_mask;
    fmask.convertTo(fmask, CV_32FC1);
    fmask *= 0.25;

    std::vector<cv::Mat> channels(3);
    channels[0] = cv::Mat(fmask.rows, fmask.cols, CV_32FC1);
    channels[1] = cv::Mat(fmask.rows, fmask.cols, CV_32FC1);
    channels[0] = cv::Scalar(0);
    channels[1] = cv::Scalar(0);
    channels[2] = fmask;

    cv::Mat final;
    cv::merge(channels, final);

    cv::Mat cref = reference_image;
    cv::cvtColor(cref, cref, CV_GRAY2BGR);

    camera_display.Activate();
    unsigned char* data = images[0].data;
    camera_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
    camera_texture.RenderToViewportFlipY();

    cv::Mat tmat = final + 0.9 * cref;
    tmat.convertTo(tmat, CV_8UC3, 255.0);

    if (target_reference_width < target_reference_height)
    {
      tmat = tmat.t();
    }

    target_display.Activate();
    unsigned char* tdata = tmat.data;
    target_texture.Upload(tdata, GL_BGR, GL_UNSIGNED_BYTE);
    target_texture.RenderToViewportFlipXFlipY();

    {
      cv::Mat mat;
      vig_values.convertTo(mat, CV_8UC1, 255.0);

      double vmin;
      double vmax;
      cv::minMaxLoc(mat, &vmin, &vmax);

      cv::cvtColor(mat, mat, CV_GRAY2BGR);

      for (int y = 0; y < mat.rows; ++y)
      {
        for (int x = 0; x < mat.cols; ++x)
        {
          mat.at<Pixel>(y, x)[0] = 255.0 * mat.at<Pixel>(y, x)[0] / vmax;
          mat.at<Pixel>(y, x)[1] = 255.0 * mat.at<Pixel>(y, x)[1] / vmax;
          mat.at<Pixel>(y, x)[2] = 255.0 * mat.at<Pixel>(y, x)[2] / vmax;
        }
      }

      vignetting_display.Activate();
      unsigned char* data = mat.data;
      vignetting_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      vignetting_texture.RenderToViewportFlipY();
    }

    {
      cv::Mat mat;
      vig_weights.convertTo(mat, CV_8UC1, 0.255);
      cv::cvtColor(mat, mat, CV_GRAY2BGR);

      for (int y = 0; y < mat.rows; ++y)
      {
        for (int x = 0; x < mat.cols; ++x)
        {
          mat.at<Pixel>(y, x) = colormap[mat.at<Pixel>(y, x)[0]];
        }
      }

      coverage_display.Activate();
      unsigned char* data = mat.data;
      coverage_texture.Upload(data, GL_RGB, GL_UNSIGNED_BYTE);
      coverage_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }

  if (!pangolin::ShouldQuit() && capture_ended)
  {
    if (!FLAGS_output_image.empty())
    {
      double vmin;
      double vmax;
      cv::minMaxLoc(vig_values, &vmin, &vmax);

      cv::Mat output;
      vig_values.convertTo(output, CV_16UC1, 65535.0 / vmax);
      cv::imwrite(FLAGS_output_image, output);
    }
  }

  // create vignetting problem from reference image set
  // create initial vignetting model
  // solve problem

  LOG(INFO) << "Success";
  return 0;
}
