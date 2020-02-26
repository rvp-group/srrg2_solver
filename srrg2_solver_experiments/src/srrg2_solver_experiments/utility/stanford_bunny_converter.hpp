#include <fstream>
#include <iostream>
#include <map>
#include <srrg_geometry/geometry_defs.h>
#include <srrg_pcl/point.h>
#include <srrg_system_utils/shell_colors.h>

namespace srrg2_solver_experiments {

  class StanfordBunnyConverter {
  public:
    StanfordBunnyConverter() = delete;
    StanfordBunnyConverter(const std::string& filename_) : _filename(filename_) {
    }
    ~StanfordBunnyConverter() {
    }

    void parsePLYFile() {
      _pointcloud.clear();
      std::ifstream file(_filename);
      if (!file.good() || !file.is_open()) {
        throw std::runtime_error("ERROR: unable to open ground truth file");
      }

      std::cerr << "converting gt file [" << FG_YELLOW(_filename) << "]" << std::endl;

      std::string line;

      size_t num_vertexes = 0;

      while (std::getline(file, line)) {
        if (line.find("element vertex") != std::string::npos) {
          std::string tmp1, tmp2;
          std::stringstream ss(line);
          ss >> tmp1 >> tmp2 >> num_vertexes;
        }
        if (line.find("end_header") != std::string::npos) {
          break;
        }
      }

      std::cerr << "num_vert " << num_vertexes << std::endl;

      size_t i = 0;
      while (i++ < num_vertexes) {
        std::getline(file, line);
        double p_x = 0, p_y = 0, p_z = 0;
        double confidence = 0;
        double intensity  = 0;
        std::stringstream ss(line);
        ss >> p_x >> p_y >> p_z >> confidence >> intensity;

        srrg2_core::Point3f p;
        p.coordinates() = srrg2_core::Vector3f(p_x, p_y, p_z);
        _pointcloud.push_back(p);
      }
    }

    const srrg2_core::Point3fVectorCloud& pointcloud() const {
      return _pointcloud;
    }

  protected:
    const std::string _filename;
    srrg2_core::Point3fVectorCloud _pointcloud;
  };
} // namespace srrg2_solver_experiments
