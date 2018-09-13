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
#include <pcalib/pcalib.h>

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
DEFINE_double(max_weight, 200, "max vignetting integration weight");
DEFINE_double(inlier_thresh, 0.05, "maximum response error for inliers");
DEFINE_int32(ransac_iters, 5000, "number of ransac iterations");

using namespace pcalib;

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PHOTOCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing HAL camera uri");
  PHOTOCALIB_ASSERT_MSG(!FLAGS_calib.empty(), "missing camera calibration");

  LOG(INFO) << "Running...";

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

  // orbbec
  // response.set_parameters(Eigen::Vector3d(0.999711, -1.37383, 1.37412));

  // // xtion
  // response.set_parameters(Eigen::Vector3d(0.506497, .0934983, 0.400005));
  response.set_parameters(Eigen::Vector3d(0.504067,  0.0248056,  0.471128));

  cv::Mat vignetting = cv::imread("/home/mike/Code/pcalib/build/apps/vignetting/xtion_vignetting_smooth.png", CV_LOAD_IMAGE_ANYDEPTH);
  vignetting.convertTo(vignetting, CV_32FC1, 1.0 / 65535.0);

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

  // create pangolin views

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

  pangolin::View& depth_display = pangolin::Display("depth");
  depth_display.SetLock(pangolin::LockLeft, pangolin::LockTop);
  depth_display.SetBounds(0.5, 1.0, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(depth_display);

  pangolin::View& normal_display = pangolin::Display("normal");
  normal_display.SetLock(pangolin::LockRight, pangolin::LockBottom);
  normal_display.SetBounds(0.0, 0.5, 0.0, 1.0, image_aspect_ratio);
  lower_stats_display.AddDisplay(normal_display);

  pangolin::GlTexture camera_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture depth_texture(width, height, GL_RGB, false, 0, GL_RED, GL_UNSIGNED_BYTE);
  pangolin::GlTexture normal_texture(width, height, GL_RGB, false, 0, GL_RGB, GL_UNSIGNED_BYTE);

  std::shared_ptr<LightProblem> problem;
  problem = std::make_shared<LightProblem>();

  bool capture_ended = false;
  bool capture_paused = false;
  pangolin::RegisterKeyPressCallback(' ', [&](){ capture_paused = !capture_paused; });
  pangolin::RegisterKeyPressCallback('[', [&](){ capture_ended  = !capture_ended; });
  pangolin::RegisterKeyPressCallback(']', [&](){ capture_ended  = !capture_ended; });

  const Eigen::Matrix3d& K = rig->cameras_[0]->K();
  Eigen::Matrix3d Kinv = Eigen::Matrix3d::Identity();

  Kinv(0, 0) =  1.0 / K(0, 0);
  Kinv(1, 1) =  1.0 / K(1, 1);
  Kinv(0, 2) = -K(0, 2) * Kinv(0, 0);
  Kinv(1, 2) = -K(1, 2) * Kinv(1, 1);

  cv::Mat depth_image(height, width, CV_8UC3);
  cv::Mat normal_image(height, width, CV_8UC3);

  depth_image = cv::Scalar(0);
  normal_image = cv::Scalar(0);

  while (!pangolin::ShouldQuit() && !capture_ended)
  {
    glClear(GL_COLOR_BUFFER_BIT);

    if (!capture_paused)
    {
      // capture raw image

      if (!raw_camera->Capture(images))
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
          const Sophus::SE3d Twc = Tcw.inverse();
          const Eigen::Vector4d Xwn(0, 0, -1, 0);
          const Eigen::Vector4d Xcn = Tcw.matrix() * Xwn;

          depth_image = cv::Scalar(0);
          normal_image = cv::Scalar(0);

          for (int y = 0; y < undistort_image.rows; ++y)
          {
            for (int x = 0; x < undistort_image.cols; ++x)
            {
              const float intensity = undistort_image.at<uint8_t>(y, x) / 255.0;

              if (intensity > 0.02 && intensity < 0.98)
              {
                const Eigen::Vector3d uvw(x + 0.5, y + 0.5, 1);
                const Eigen::Vector3d Xcp = Kinv * uvw;

                const Eigen::Vector3d Xwp = Twc * Xcp;
                const Eigen::Vector3d origin = Tcw.inverse().translation();
                const Eigen::Vector3d dir = Xwp - origin;

                if (std::abs(dir[2]) < 1E-8)
                {
                  const uint8_t i = undistort_image.at<uint8_t>(y, x);
                  undistort_image.at<uint8_t>(y, x) = 0.3 * i;
                  continue;
                }

                const double t = -origin[2] / dir[2];
                const Eigen::Vector3d p = origin + t * dir;

                if (p[0] < 0 || p[0] > target_physical_size[0] ||
                    p[1] < 0 || p[1] > target_physical_size[1])
                {
                  const uint8_t i = undistort_image.at<uint8_t>(y, x);
                  undistort_image.at<uint8_t>(y, x) = 0.3 * i;
                  continue;
                }

                const int tx = p[0] * target_scaling[0];
                const int ty = p[1] * target_scaling[1];
                const bool mask = target_mask.at<unsigned char>(ty, tx);

                if (!mask)
                {
                  const uint8_t i = undistort_image.at<uint8_t>(y, x);
                  undistort_image.at<uint8_t>(y, x) = 0.3 * i;
                  continue;
                }

                double irradiance = response(intensity);
                irradiance /= vignetting.at<float>(y, x);

                const Eigen::Vector3d Xcp_t = t * Xcp.normalized();
                const Eigen::Vector3d Xcn_t(Xcn[0], Xcn[1], Xcn[2]);

                if (x % 8 == 0 && y % 8 == 0)
                {
                  LightSample sample;
                  sample.irradiance = irradiance;
                  sample.Xcn = Xcn_t;
                  sample.Xcp = Xcp_t;

                  problem->samples.push_back(sample);
                }

                Light<double> light;
                // light.set_intensity(0.40169);
                // light.set_position(0.0241557, 0.0783387, -0.0767418);
                light.set_intensity(0.807638);
                light.set_position(-0.00496583, 0.0616614, -0.167348);
                const double shading = light.GetShading(Xcp_t, Xcn_t);
                const double irr = shading;

                Pixel normal_color;
                // normal_color[0] = uint8_t(127.5 * (Xcn[0] + 1));
                // normal_color[1] = uint8_t(127.5 * (Xcn[1] + 1));
                // normal_color[2] = uint8_t(127.5 * (Xcn[2] + 1));
                normal_color[0] = std::max(0.0, std::min(255.0, 255 * irr));
                normal_color[1] = std::max(0.0, std::min(255.0, 255 * irr));
                normal_color[2] = std::max(0.0, std::min(255.0, 255 * irr));
                depth_image.at<Pixel>(y, x) = colormap[uint8_t(std::max(0.0, std::min(255.0, 255 * Xcp_t[2] / 1)))];
                normal_image.at<Pixel>(y, x) = normal_color;

                images[0].at<uint8_t>(y, x) = std::max(0.0, std::min(255.0, 255 * irradiance));
              }
            }
          }
        }
      }
    }

    {
      cv::Mat mat;
      cv::cvtColor(undistort_image, mat, CV_GRAY2BGR);

      camera_display.Activate();
      camera_texture.Upload(mat.data, GL_RGB, GL_UNSIGNED_BYTE);
      camera_texture.RenderToViewportFlipY();
    }

    {
      depth_display.Activate();
      depth_texture.Upload(depth_image.data, GL_RGB, GL_UNSIGNED_BYTE);
      depth_texture.RenderToViewportFlipY();
    }

    {
      normal_display.Activate();
      normal_texture.Upload(normal_image.data, GL_RGB, GL_UNSIGNED_BYTE);
      normal_texture.RenderToViewportFlipY();
    }

    pangolin::FinishFrame();
  }

  if (!pangolin::ShouldQuit())
  {
    Light<double> light;
    light.set_intensity(1.0);
    light.set_position(0.0, 0.0, 0.0);

    LightProblemSolver solver(problem);
    solver.set_inlier_threshold(FLAGS_inlier_thresh);
    solver.set_ransac_iterations(FLAGS_ransac_iters);
    solver.Solve(light);

    LOG(INFO) << "light intensity: " << light.intensity();
    LOG(INFO) << "light position: " << light.position();
  }

  // visualize results

  LOG(INFO) << "Success";
  return 0;
}
