#include <gtest/gtest.h>

#include "srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h"
#include "srrg_solver_extras/types_3d_ad/se3_point2point_euler_error_factor_ad.h"

#include <srrg_messages/instances.h>
#include <srrg_pcl/point.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_test/test_helper.hpp>
// ds measurement adaption
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>

#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/icp.h>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;

const char* banner[] = {"This program a rgb/d pair data from ICL and performs experiments", 0};

static std::string rgb_file   = "";
static std::string depht_file = "";
static std::string output_dir = "";

srrg2_core::Point3fVectorCloud fixed_cloud;
srrg2_core::Point3fVectorCloud moving_cloud;
CorrespondenceVector correspondences;
size_t n_iterations = 10;
Isometry3f T;

int main(int argc, char** argv) {
  srrgInit(argc, argv);
  ParseCommandLine command_line_parser(argv, banner);
  ArgumentString filepath_image_depth(
    &command_line_parser, "d", "image-depth", "file path for depth image", "");
  ArgumentString filepath_image_rgb(
    &command_line_parser, "i", "image-rgb", "file path for RGB image", "");
  ArgumentString argument_output_directory(
    &command_line_parser, "od", "output directory", "Results directory", "");
  ArgumentInt argument_iterations(
    &command_line_parser, "it", "iterations", "number of iterations", 10);

  command_line_parser.parse();

  if (filepath_image_depth.value().empty()) {
    std::cerr << "ERROR: rgb input file is empty (set with -i)" << std::endl;
    return 0;
  }
  if (filepath_image_rgb.value().empty()) {
    std::cerr << "ERROR: depht input file is empty (set with -d)" << std::endl;
    return 0;
  }
  if (argument_output_directory.value().empty()) {
    std::cerr << "ERROR: out dir is empty (set with -od)" << std::endl;
    return 0;
  }

  rgb_file   = filepath_image_rgb.value();
  depht_file = filepath_image_depth.value();
  output_dir = argument_output_directory.value();
  if (output_dir.back() != '/') {
    output_dir.push_back('/');
  }
  n_iterations = argument_iterations.value();
  std::cerr << "n iterations: " << n_iterations << std::endl;

  if (!srrg2_core::isAccessible(rgb_file)) {
    std::cerr << "ERROR: " << rgb_file << " is inaccessible" << std::endl;
    return 0;
  }
  if (!srrg2_core::isAccessible(depht_file)) {
    std::cerr << "ERROR: " << depht_file << " is inaccessible" << std::endl;
    return 0;
  }
  if (!srrg2_core::isAccessible(output_dir)) {
    std::cerr << "ERROR: " << output_dir << " is inaccessible" << std::endl;
    return 0;
  }

  // ds read opencv images from disk
  const cv::Mat image_depth_opencv(
    cv::imread(filepath_image_depth.value(), CV_LOAD_IMAGE_ANYDEPTH));
  const cv::Mat image_rgb_opencv(cv::imread(filepath_image_rgb.value(), CV_LOAD_IMAGE_UNCHANGED));
  const cv::Mat image_grey_opencv(cv::imread(filepath_image_rgb.value(), CV_LOAD_IMAGE_GRAYSCALE));

  ImageUInt16 image_depth;
  image_depth.fromCv(image_depth_opencv);
  ImageUInt8 image_grey;
  image_grey.fromCv(image_grey_opencv);

  // ds this matrix is hardcoded for the example images in (srrg2_proslam/test_data/)
  Matrix3f K;
  K << 481.2, 0, 319.5, 0, -481, 239.5, 0, 0, 1;

  // ds allocate an unprojector
  PointUnprojector_<PinholeUnprojection, Point3fVectorCloud> unprojector;
  unprojector.setCameraMatrix(K);
  unprojector.param_range_min.setValue(0.1f);
  unprojector.param_range_max.setValue(10.0f);

  // ds convert input to proper format TODO avoid this?
  ImageFloat image_depth_float;
  image_depth.convertTo(image_depth_float, 1e-3);
  ImageFloat image_grey_float;
  image_grey.convertTo(image_grey_float, 1.f / 255);

  // ds point cloud to be generated from RGB-D data through unproject (preallocate memory)
  fixed_cloud.resize(image_depth_float.size());

  // ds compute unprojection for all points
  const size_t number_of_points =
    unprojector.compute(fixed_cloud.data(), image_depth_float /*, image_grey_float*/);
  fixed_cloud.resize(number_of_points);
  std::cerr << "# unprojected points: " << number_of_points << std::endl;

  moving_cloud = fixed_cloud;
  correspondences.reserve(fixed_cloud.size());

  for (size_t i = 0; i < fixed_cloud.size(); ++i) {
    correspondences.emplace_back(Correspondence(i, i));
  }

  //  srand((unsigned int) time(0));
  const Vector3f t            = Vector3f::Random() * 10;
  const Vector3f euler_angles = Vector3f::Random() * M_PI;
  T.setIdentity();
  T.linear()      = geometry3d::a2r(euler_angles);
  T.translation() = t;

  std::cerr << FG_YELLOW(T.matrix()) << std::endl;

  moving_cloud.transformInPlace<Isometry>(T);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "tests.hpp"
