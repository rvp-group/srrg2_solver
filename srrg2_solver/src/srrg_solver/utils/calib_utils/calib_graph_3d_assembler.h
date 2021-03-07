#pragma once
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_calib/sensor3d_extrinsic_pose_motion_calib_ad.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sink_base.h>

namespace srrg2_solver {

  class CalibGraph3DAssembler : public MessageSinkBase {
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
          sensor_odom_topic,
          "topic of the sensor odometry, expressed in our sensor ref system",
          "/camera/odom/sample",
          0);

    CalibGraph3DAssembler();
    virtual ~CalibGraph3DAssembler();
    bool putMessage(BaseSensorMessagePtr msg_) override;
    void reset();
    FactorGraphPtr graph();

    bool cmdSaveGraph(std::string& response, const std::string& filename);

  protected:
    FactorGraphPtr _graph                    = nullptr;
    Eigen::Isometry3f _last_gt_odom_pose     = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f _last_sensor_odom_pose = Eigen::Isometry3f::Identity();
    int _factor_count                        = 0;
  };
} // namespace srrg2_solver
