#include <smpl/heuristic/joint_dist_weighted_heuristic.h>

// standard includes
#include <cmath>

#include <smpl/console/console.h>

namespace smpl {
    double JointDistWeightedHeuristic::computeJointDistance(
      RobotState const & s,
      RobotState const & t
    ) const {
        unsigned int const prismatic_joint_count = s.size() - 7;
        double constexpr prismatic_joint_weight = 10.0;

        double dsum = 0.0;
        for (size_t i = 0; i < s.size(); ++i) {
            double const dj = (s[i] - t[i]);
            dsum += dj * dj;

            if (i < prismatic_joint_count) {
                dsum *= prismatic_joint_weight;
            }
        }

        return std::sqrt(dsum);
    }
}  // namespace smpl
