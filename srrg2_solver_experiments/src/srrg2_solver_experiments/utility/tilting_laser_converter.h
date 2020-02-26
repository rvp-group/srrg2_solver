#include <fstream>
#include <iostream>
#include <map>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_solver_experiments {

  class ETHTiltingLaserConverter {
  public:
    using GTDataset = std::map<size_t,
                               srrg2_core::Isometry3f,
                               std::less<size_t>,
                               Eigen::aligned_allocator<std::pair<size_t, srrg2_core::Isometry3f>>>;
    using PCDataset =
      std::map<size_t,
               srrg2_core::Point3fVectorCloud,
               std::less<size_t>,
               Eigen::aligned_allocator<std::pair<size_t, srrg2_core::Point3fVectorCloud>>>;

    ETHTiltingLaserConverter() = delete;
    ETHTiltingLaserConverter(const std::string& dir_) : _dir(dir_) {
    }
    ~ETHTiltingLaserConverter() {
      _gt_dataset.clear();
    }

    inline void setGTFilename(const std::string& gt_filename_) {
      _gt_filename = gt_filename_;
    }

    inline void processETHTiltingLaserDirectory() {
      processETHGroundTruth(_gt_filename);
      processETHPointClouds();
    }

    inline const GTDataset& getGroundTruth() const {
      return _gt_dataset;
    }

    inline const PCDataset& getPointClouds() const {
      return _pc_dataset;
    }

  protected:
    void processETHGroundTruth(const std::string& filename_) {
      std::string filename = filename_;
      if (_dir.back() != '/') {
        filename.insert(0, "/");
      }
      filename = _dir + filename;

      std::ifstream ground_truth_file(filename);
      if (!ground_truth_file.good() || !ground_truth_file.is_open()) {
        throw std::runtime_error("ERROR: unable to open ground truth file");
      }

      std::cerr << "converting gt file [" << FG_YELLOW(filename) << "]" << std::endl;

      std::string line;
      // srrg skip the tag line
      std::getline(ground_truth_file, line);
      size_t pose_id;
      srrg2_core::Isometry3f T;
      while (std::getline(ground_truth_file, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');
        groundTruthlineParser(line, pose_id, T);
        _gt_dataset.insert(std::make_pair(pose_id, T));
      }
      ground_truth_file.close();
      std::cerr << "loaded ground truth poses: " << _gt_dataset.size() << std::endl;
    }

    void processETHPointClouds() {
      //      const size_t point_clouds_num = 36;

      for (size_t i = 0; i < 1; ++i) {
        std::string filename = "PointCloud" + std::to_string(i) + ".csv";
        if (_dir.back() != '/') {
          filename.insert(0, "/");
        }
        filename = _dir + filename;
        std::ifstream filepath(filename);
        std::cerr << "converting " << FG_YELLOW(filename) << std::endl;
        srrg2_core::Point3fVectorCloud pc;

        std::string line;
        std::getline(filepath, line);
        while (std::getline(filepath, line)) {
          std::replace(line.begin(), line.end(), ',', ' ');
          pointCloudLineParser(line, pc);
        }

        std::cerr << "PointCloud " << i << " has " << pc.size() << " points" << std::endl;

        _pc_dataset.insert(std::make_pair(i, pc));
        filepath.close();
      }
    }

    inline void pointCloudLineParser(const std::string& line_,
                                     srrg2_core::Point3fVectorCloud& pc_) {
      double timestamp = 0;
      double p_x = 0, p_y = 0, p_z = 0;
      double intensity        = 0;
      size_t scan_id_2d       = 0;
      size_t point_id_in_scan = 0;
      std::stringstream ss(line_);
      ss >> timestamp;
      ss >> p_x >> p_y >> p_z;
      ss >> intensity >> scan_id_2d >> point_id_in_scan;

      srrg2_core::Point3f p;
      p.coordinates() = srrg2_core::Vector3f(p_x, p_y, p_z);
      pc_.push_back(p);
    }

    inline void
    groundTruthlineParser(const std::string& line_, size_t& pose_id_, srrg2_core::Isometry3f& T_) {
      double timestamp = 0;
      double t_x = 0, t_y = 0, t_z = 0;
      // clang-format off
      double r_00 = 0, r_01 = 0, r_02 = 0,
             r_10 = 0, r_11 = 0, r_12 = 0,
             r_20 = 0, r_21 = 0, r_22 = 0;
      // clang-format on
      srrg2_core::Matrix3f m;

      std::stringstream ss(line_);
      ss >> pose_id_ >> timestamp;
      ss >> r_00 >> r_01 >> r_02 >> t_x;
      ss >> r_10 >> r_11 >> r_12 >> t_y;
      ss >> r_20 >> r_21 >> r_22 >> t_z;

      // clang-format off
      m << r_00, r_01, r_02,
           r_10, r_11, r_12,
           r_20, r_21, r_22;
      // clang-format on

      T_.translation() = srrg2_core::Vector3f(t_x, t_y, t_z);
      T_.linear()      = m;
    }

    std::string _dir;
    GTDataset _gt_dataset;

    PCDataset _pc_dataset;

    std::string _gt_filename = "pose_scanner_leica.csv";
  };

} // namespace srrg2_solver_experiments
