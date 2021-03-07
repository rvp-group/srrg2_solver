#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_calib/differential_drive_odom_sensor2d_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_calib/sensor2d_extrinsic_pose_motion_calib_ad.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sink_base.h>

namespace srrg2_solver {

  class CalibGraph2DAssembler : public MessageSinkBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat, min_translation, "minimum translation to deploy measurements", 0.3, 0);
    PARAM(PropertyFloat, min_rotation, "minimum rotation to deploy measurements", 0.3, 0);
    PARAM(PropertyString,
          gt_odom_topic,
          "topic for the gt odometry of the sensor motion",
          "/tracker_odom",
          0);
    PARAM(PropertyString,
          robot_odom_topic,
          "topic of the robot odometry, blind without not refined by any sensors",
          "/odom",
          0);
    PARAM(PropertyBool,
          use_odom,
          "if true use directly robot odometry, only sensor pose wrt robot calibration",
          false,
          0);

    PARAM(PropertyString, joint_state_topic, "topic for joint state", "/joint_state", 0);
    PARAM(PropertyString,
          left_wheel_joint,
          "identifier for the left wheel joint",
          "front_left_wheel",
          0);
    PARAM(PropertyString,
          right_wheel_joint,
          "identifier for the right wheel joint",
          "front_right_wheel",
          0);

    CalibGraph2DAssembler();
    virtual ~CalibGraph2DAssembler();
    bool putMessage(BaseSensorMessagePtr msg_) override;
    void reset();
    FactorGraphPtr graph();

    bool cmdSaveGraph(std::string& response, const std::string& filename);

  protected:
    bool _calibrateWithOdom(srrg2_core::MessagePackPtr pack_);
    bool _calibrateWithTicks(srrg2_core::MessagePackPtr pack_);

    FactorGraphPtr _graph             = nullptr;
    Eigen::Isometry2f _last_node_pose = Eigen::Isometry2f::Identity();
    Eigen::Isometry2f _last_odom_pose = Eigen::Isometry2f::Identity();
    Eigen::Vector2f _last_ticks       = Eigen::Vector2f::Zero();
    int _factor_count                 = 0;
  };
} // namespace srrg2_solver
