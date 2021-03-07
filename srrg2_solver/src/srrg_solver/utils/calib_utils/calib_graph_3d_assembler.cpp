#include "calib_graph_3d_assembler.h"
#include <srrg_config/configurable_command.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_system_utils/system_utils.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  CalibGraph3DAssembler::CalibGraph3DAssembler() {
    addCommand(new ConfigurableCommand_<CalibGraph3DAssembler,
                                        typeof(&CalibGraph3DAssembler::cmdSaveGraph),
                                        std::string,
                                        std::string>(
      this, "saveGraph", "saves a graph to a json file", &CalibGraph3DAssembler::cmdSaveGraph));
    std::cerr << param_sensor_odom_topic.value() << std::endl;
    std::cerr << param_gt_odom_topic.value() << std::endl;

    reset();
  }

  CalibGraph3DAssembler::~CalibGraph3DAssembler() {
  }

  bool CalibGraph3DAssembler::putMessage(BaseSensorMessagePtr msg_) {
    srrg2_core::MessagePackPtr pack = std::dynamic_pointer_cast<srrg2_core::MessagePack>(msg_);
    if (!pack) {
      return false;
    }

    OdometryMessagePtr gt_odom, sensor_odom;
    for (BaseSensorMessagePtr m : pack->messages) {
      std::cerr << m->topic.value() << std::endl;
      if (m->topic.value() == param_gt_odom_topic.value()) {
        std::cerr << param_gt_odom_topic.value() << std::endl;
        gt_odom = std::dynamic_pointer_cast<OdometryMessage>(m);
      }
      if (m->topic.value() == param_sensor_odom_topic.value()) {
        sensor_odom = std::dynamic_pointer_cast<OdometryMessage>(m);
        std::cerr << param_sensor_odom_topic.value() << std::endl;
      }
      if (!gt_odom || !sensor_odom) {
        continue;
      }
    }

    Eigen::Isometry3f sensor_odom_pose = sensor_odom->pose.value();
    Eigen::Isometry3f gt_odom_pose     = gt_odom->pose.value();
    Eigen::Isometry3f measurement      = _last_sensor_odom_pose.inverse() * sensor_odom_pose;
    srrg2_core::Vector6f delta         = geometry3d::t2ta(measurement);

    bool make_factor = (!_factor_count || delta.head<2>().norm() > param_min_translation.value() ||
                        fabs(delta(5)) > param_min_rotation.value());
    if (!make_factor) {
      return false;
    }
    if (!_factor_count) {
      reset();
      _last_gt_odom_pose     = gt_odom_pose;
      _last_sensor_odom_pose = sensor_odom_pose;
      ++_factor_count;
      return true;
    }

    std::shared_ptr<SE3SensorExtrinsicPoseMotionCalibAD> factor(
      new SE3SensorExtrinsicPoseMotionCalibAD);
    factor->setGraphId(_factor_count + 1000);
    factor->setVariableId(0, 0);
    factor->setFrom(_last_gt_odom_pose);
    factor->setTo(gt_odom_pose);
    factor->setMeasurement(measurement);
    _graph->addFactor(factor);

    std::cerr << "sensor displacement: " << delta.transpose() << std::endl;
    std::cerr << "gt displacement: "
              << geometry3d::t2ta(_last_gt_odom_pose.inverse() * gt_odom_pose).transpose()
              << std::endl;

    ++_factor_count;
    _last_sensor_odom_pose = sensor_odom_pose;
    _last_gt_odom_pose     = gt_odom_pose;

    return true;
  }

  void CalibGraph3DAssembler::reset() {
    _factor_count = 0;
    if (!_graph) {
      _graph = FactorGraphPtr(new FactorGraph);
    } else {
      _graph->clear();
    }

    std::shared_ptr<VariableSE3QuaternionRightAD> sensor_pose(new VariableSE3QuaternionRightAD);
    sensor_pose->setZero();
    sensor_pose->setGraphId(0);
    _graph->addVariable(sensor_pose);
  }

  FactorGraphPtr CalibGraph3DAssembler::graph() {
    return _graph;
  }

  bool CalibGraph3DAssembler::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = className() + "| saving graph to file [" + filename + "]";
    if (!_graph) {
      response += " No Graph, Aborting";
      return false;
    }
    graph()->write(filename);
    return true;
  }

} // namespace srrg2_solver
