#ifndef SMPL_JOINT_DIST_WEIGHTED_HEURISTIC_H
#define SMPL_JOINT_DIST_WEIGHTED_HEURISTIC_H

// project includes
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {
    class JointDistWeightedHeuristic : public JointDistHeuristic {
      private:
        constexpr static double FIXED_POINT_RATIO = 1000.0;

        ExtractRobotStateExtension * m_ers = nullptr;

        double computeJointDistance(RobotState const & s, RobotState const & t) const;
    };
}  // namespace smpl

#endif
