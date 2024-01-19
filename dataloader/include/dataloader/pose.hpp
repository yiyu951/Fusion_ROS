#ifndef FUSION_ROS_POSE_HPP
#define FUSION_ROS_POSE_HPP


#include "dataloader/types.hpp"

namespace Fusion {


    class POSE {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        POSE(double timestamp, const Mat3 &R, const Vec3 &t)
                : timestamp_(timestamp), pose_(R, t) {
        }

        POSE(double timestamp, const SE3 &pose)
                : timestamp_(timestamp), pose_(pose) {
        }

    public:
        double timestamp_;
        SE3 pose_;
    };
    using POSEPtr = std::shared_ptr<POSE>;
}
#endif //FUSION_ROS_POSE_HPP
