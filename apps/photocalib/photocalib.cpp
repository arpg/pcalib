#include <gflags/gflags.h>
#include <glog/logging.h>

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  LOG(INFO) << "Running...";

  // ==== INITIAL SETUP ====

  // read camera flags
  // create camera object (raw feed)
  // check if exposure available
  // read camera projection
  // read camera distortion

  // ==== RESPONSE CALIBRATION ====

  // check if response given
    // optionally use as initialization
    // optionally skip step

  // capture different exposures
    // adjust to hit mean levels
    // capture several and average
    // record final exposure value
    // record intensity coverage

  // build optimization
    // read desired response model
      // polynomial degree
      // shared or individual per channel
    // optionally read provided initial response parameters
    // add parameter blocks
      // polynomial coefficient vector
    // add residuals
      // exp0, int0, exp1, int1

  // solve optimization
    // read RANSAC iteration limit flag
    // read RANSAC outlier threshold flag
    // perform RANSAC
      // select samples (count = polynomial order)
      // perform linear system solve
      // create inlier list
      // check if better inliear count found
    // perform final solve
      // initialize with RANSAC estimate
      // only use RANSAC inliers
      // create and solve ceres problem

  // visualize results
    // show captured image w/ low exposure
    // show captured image w/ high exposure
    // show low to high scaling assuming linear response
    // show low to high scaling assuming estimated response
    // plot response curves

  // ==== VIGNETTING CALIBRATION ====

  // check if vignetting was requested

  // capture vignetting data
    // attempt to detect target
    // find proper exposure for images
    // detect calibu target in undistorted image
    // project target into fixed frame given extrincis & intrinsnics
    // build patch buffers
      // randomly sample and add data to patch buffers
      // tally image patch counts
      // visualize live image
      // visualize fixed frame image
      // visualize tally count heat map
      // stop on button press, good tallies, or fixed frame count
    // build pixel averages
      // assume calibu target is uniformly illuminated (and intenisty = 1)
      // average inv intensity for each projected pixel
      // perhaps perform additional post-process smoothing
        // be light your neighbors

  // build optimization
    // read desired vignetting model
      // polynomial degree
    // optionally read provided initial vignetting parameters
    // add parameter blocks
      // polynomial coefficient vector
    // add residuals
      // convert intensities to irradiance
      // irr, u, v

  // solve optimization
    // optionally initialize with given estimate
    // optionally initialize with uniform vignetting
    // create and solve ceres problem
    // use soft L1 loss function

  // visualize results
    // show capture image w/ vignetting
    // show capture image w/o vignetting
    // show vignetting falloff w/ solid white image
    // plot horizontal and vertical falloff lines

  // ==== WRITE OUTPUT ====

  LOG(INFO) << "Success";
  return 0;
}