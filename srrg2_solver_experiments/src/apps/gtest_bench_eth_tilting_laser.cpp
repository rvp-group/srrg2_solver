#include <gtest/gtest.h>

#include "srrg2_solver_experiments/utility/tilting_laser_converter.h"

#include "srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h"
#include "srrg_solver_extras/types_3d_ad/se3_point2point_euler_error_factor_ad.h"

#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_test/test_helper.hpp>

#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/icp.h>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_solver_extras;
using namespace srrg2_solver_experiments;

const char* banner[] = {
  "This program takes ETH Tilting Laser formatted PointClouds and performs experiments",
  0};

static std::string input_dir  = "";
static std::string output_dir = "";

Point3fVectorCloud fixed_cloud;
Point3fVectorCloud moving_cloud;
CorrespondenceVector correspondences;
size_t n_iterations = 0;
Isometry3f T;

int main(int argc, char** argv) {
  srrgInit(argc, argv);
  ParseCommandLine command_line_parser(argv, banner);
  ArgumentString argument_input_directory(
    &command_line_parser, "id", "input-directory", "Point cloud directory of the ETH dataset", "");
  ArgumentString argument_output_directory(
    &command_line_parser, "od", "output-directory", "Results directory", "");
  ArgumentInt argument_iterations(
    &command_line_parser, "it", "iterations", "number of iterations", 10);

  command_line_parser.parse();

  if (argument_input_directory.value().empty()) {
    std::cerr << "ERROR: input dir is empty (set with -id)" << std::endl;
    return 0;
  }
  if (argument_output_directory.value().empty()) {
    std::cerr << "ERROR: out dir is empty (set with -od)" << std::endl;
    return 0;
  }

  input_dir  = argument_input_directory.value();
  output_dir = argument_output_directory.value();
  if (input_dir.back() != '/') {
    input_dir.push_back('/');
  }
  if (output_dir.back() != '/') {
    output_dir.push_back('/');
  }
  n_iterations = argument_iterations.value();
  std::cerr << "n iterations: " << n_iterations << std::endl;

  if (!srrg2_core::isAccessible(output_dir)) {
    std::cerr << "ERROR: " << output_dir << " is inaccessible" << std::endl;
    return 0;
  }

  ETHTiltingLaserConverter conv(argument_input_directory.value());
  conv.processETHTiltingLaserDirectory();
  const auto& pcs = conv.getPointClouds();
  fixed_cloud     = pcs.at(0);

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
