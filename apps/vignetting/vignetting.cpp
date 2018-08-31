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

using namespace photocalib;

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  PHOTOCALIB_ASSERT_MSG(!FLAGS_cam.empty(), "missing HAL camera uri");
  PHOTOCALIB_ASSERT_MSG(!FLAGS_calib.empty(), "missing camera calibration");

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
  response.set_parameters(Eigen::Vector3d(0.999711, -1.37383, 1.37412));

  const int width = raw_camera->Width();
  const int height = raw_camera->Height();
  const int pitch = width * sizeof(unsigned char);

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

  while (true)
  {
    raw_camera->Capture(images);
    cv::cvtColor(images[0], raw_image, CV_BGR2GRAY);

    unsigned char* dst = undistort_image.data;
    const unsigned char* src = raw_image.data;
    calibu::Rectify<unsigned char>(undistort_lookup, src, dst, width, height);

    processing.Process(undistort_image.data, width, height, pitch);
    finder.Find(processing);

    target_found = target->FindTarget(processing, finder.Conics(), ellipse_map);

    if (!target_found)
    {
      continue;
    }

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> ellipses(finder.Conics().size());

    for (size_t i = 0; i < finder.Conics().size(); ++i)
    {
      ellipses[i] = finder.Conics()[i].center;
    }

    Sophus::SE3d Tcw;
    const int ransac_iterations = 0;
    const float ransac_tolerance = 0.0f;

    calibu::PosePnPRansac(rig->cameras_[0], ellipses, target->Circles3D(),
        ellipse_map, ransac_iterations, ransac_tolerance, &Tcw);

    const double error = calibu::ReprojectionErrorRMS(rig->cameras_[0], Tcw,
        target->Circles3D(), ellipses, ellipse_map);

    if (error > 2.0)
    {
      continue;
    }

    Eigen::Vector2d target_scaling;
    target_scaling[0] = target_reference_width  / target_physical_size[0];
    target_scaling[1] = target_reference_height / target_physical_size[1];

    reference_image = cv::Scalar(0);

    cv::Mat new_mask = target_mask.clone();

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

    cv::imshow("Live Image", undistort_image);
    cv::imshow("Project Image", final + 0.9 * cref);
    const int key = cv::waitKey(33);
    if (key == 27) break;
  }

  // create vignetting problem from reference image set
  // create initial vignetting model
  // solve problem

  LOG(INFO) << "Success";
  return 0;
}
