#include "calib_graph_2d_assembler.h"
#include <srrg_config/configurable_command.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/messages/joints_message.h>
#include <srrg_messages/messages/odometry_message.h>
#include <srrg_system_utils/system_utils.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  CalibGraph2DAssembler::CalibGraph2DAssembler() {
    addCommand(new ConfigurableCommand_<CalibGraph2DAssembler,
                                        typeof(&CalibGraph2DAssembler::cmdSaveGraph),
                                        std::string,
                                        std::string>(
      this, "saveGraph", "saves a graph to a json file", &CalibGraph2DAssembler::cmdSaveGraph));
    reset();
  }

  CalibGraph2DAssembler::~CalibGraph2DAssembler() {
  }

  bool CalibGraph2DAssembler::_calibrateWithTicks(srrg2_core::MessagePackPtr pack_) {
    OdometryMessagePtr gt_odom;
    JointsMessagePtr joint_state;
    JointEvent* left_event  = nullptr;
    JointEvent* right_event = nullptr;
    for (BaseSensorMessagePtr m : pack_->messages) {
      if (m->topic.value() == param_gt_odom_topic.value()) {
        gt_odom = std::dynamic_pointer_cast<OdometryMessage>(m);
      }
      if (m->topic.value() == param_joint_state_topic.value()) {
        joint_state = std::dynamic_pointer_cast<JointsMessage>(m);
      }
      if (!gt_odom || !joint_state) {
        continue;
      }
      for (size_t i = 0; i < joint_state->joint_events.size(); ++i) {
        JointEvent* event = &joint_state->joint_events.value(i);
        if (!event) {
          continue;
        }
        if (event->identifier() == param_left_wheel_joint.value()) {
          left_event = event;
        }
        if (event->identifier() == param_right_wheel_joint.value()) {
          right_event = event;
        }
      }
      if (!left_event) {
        continue;
      }
      if (!right_event) {
        continue;
      }
    }
    Eigen::Isometry2f pose_2d     = geometry3d::get2dFrom3dPose(gt_odom->pose.value());
    Eigen::Isometry2f measurement = _last_node_pose.inverse() * pose_2d;
    Eigen::Vector3f delta         = geometry2d::t2v(measurement);
    Eigen::Vector2f ticks(left_event->position(), right_event->position());
    bool make_factor = (!_factor_count || delta.head<2>().norm() > param_min_translation.value() ||
                        fabs(delta(2)) > param_min_rotation.value());
    if (!make_factor) {
      return false;
    }
    if (!_factor_count) {
      reset();
      _last_ticks     = ticks;
      _last_node_pose = pose_2d;
      ++_factor_count;
      return true;
    }
    std::shared_ptr<DifferentialDriveOdomSensor2DErrorFactorAD> factor(
      new DifferentialDriveOdomSensor2DErrorFactorAD);
    factor->setGraphId(_factor_count + 1000);
    factor->setVariableId(0, 0);
    factor->setVariableId(1, 1);
    Vector2f delta_ticks = ticks - _last_ticks;
    delta_ticks(0)       = atan2(sin(delta_ticks(0)), cos(delta_ticks(0)));
    delta_ticks(1)       = atan2(sin(delta_ticks(1)), cos(delta_ticks(1)));
    factor->setTicks(delta_ticks);
    factor->setMeasurement(measurement);
    _graph->addFactor(factor);
    ++_factor_count;
    _last_ticks     = ticks;
    _last_node_pose = pose_2d;
    std::cerr << "delta_ticks: " << delta_ticks.transpose() << std::endl;
    std::cerr << "displacement: " << delta.transpose() << std::endl;

    return true;
  }

  bool CalibGraph2DAssembler::_calibrateWithOdom(srrg2_core::MessagePackPtr pack_) {
    OdometryMessagePtr gt_odom, odom;
    for (BaseSensorMessagePtr m : pack_->messages) {
      if (m->topic.value() == param_gt_odom_topic.value()) {
        gt_odom = std::dynamic_pointer_cast<OdometryMessage>(m);
      }
      if (m->topic.value() == param_robot_odom_topic.value()) {
        odom = std::dynamic_pointer_cast<OdometryMessage>(m);
      }
      if (!gt_odom || !odom) {
        continue;
      }
    }

    Eigen::Isometry2f odom_pose_2d = geometry3d::get2dFrom3dPose(odom->pose.value());
    Eigen::Isometry2f gt_pose_2d   = geometry3d::get2dFrom3dPose(gt_odom->pose.value());
    Eigen::Isometry2f measurement  = _last_node_pose.inverse() * gt_pose_2d;
    Eigen::Vector3f delta          = geometry2d::t2v(measurement);

    bool make_factor = (!_factor_count || delta.head<2>().norm() > param_min_translation.value() ||
                        fabs(delta(2)) > param_min_rotation.value());
    if (!make_factor) {
      return false;
    }
    if (!_factor_count) {
      reset();
      _last_odom_pose = odom_pose_2d;
      _last_node_pose = gt_pose_2d;
      ++_factor_count;
      return true;
    }

    std::shared_ptr<SE2SensorExtrinsicPoseMotionCalibAD> factor(
      new SE2SensorExtrinsicPoseMotionCalibAD);
    factor->setGraphId(_factor_count + 1000);
    factor->setVariableId(0, 0);
    factor->setFrom(_last_odom_pose);
    factor->setTo(odom_pose_2d);
    factor->setMeasurement(measurement);
    _graph->addFactor(factor);

    std::cerr << "odom displacement: "
              << geometry2d::t2v(_last_odom_pose.inverse() * odom_pose_2d).transpose() << std::endl;
    std::cerr << "gt displacement: " << delta.transpose() << std::endl;

    ++_factor_count;
    _last_odom_pose = odom_pose_2d;
    _last_node_pose = gt_pose_2d;

    return true;
  }

  bool CalibGraph2DAssembler::putMessage(BaseSensorMessagePtr msg_) {
    srrg2_core::MessagePackPtr pack = std::dynamic_pointer_cast<srrg2_core::MessagePack>(msg_);
    if (!pack) {
      return false;
    }

    bool is_good = false;
    if (param_use_odom.value()) {
      is_good = _calibrateWithOdom(pack);
    } else {
      is_good = _calibrateWithTicks(pack);
    }
    return is_good;
  }

  void CalibGraph2DAssembler::reset() {
    _factor_count = 0;
    if (!_graph) {
      _graph = FactorGraphPtr(new FactorGraph);
    } else {
      _graph->clear();
    }

    std::shared_ptr<VariableSE2RightAD> sensor_pose(new VariableSE2RightAD);
    sensor_pose->setZero();
    if (param_use_odom.value()) {
      sensor_pose->setGraphId(0);
    } else {
      std::shared_ptr<VariablePoint3AD> kinematic_params(new VariablePoint3AD);
      kinematic_params->setGraphId(0);
      kinematic_params->setEstimate(Eigen::Vector3f(-1, 1, 0.5));
      sensor_pose->setGraphId(1);
      _graph->addVariable(kinematic_params);
    }
    _graph->addVariable(sensor_pose);
  }

  FactorGraphPtr CalibGraph2DAssembler::graph() {
    return _graph;
  }

  bool CalibGraph2DAssembler::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = className() + "| saving graph to file [" + filename + "]";
    if (!_graph) {
      response += " No Graph, Aborting";
      return false;
    }
    graph()->write(filename);
    return true;
  }

} // namespace srrg2_solver
