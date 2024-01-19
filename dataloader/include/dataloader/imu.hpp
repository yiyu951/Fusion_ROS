#ifndef FUSION_ROS_IMU_HPP
#define FUSION_ROS_IMU_HPP

#include "dataloader/types.hpp"
#include <memory>

namespace Fusion {
    class IMU {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        IMU(double timestamp, const Vec3 &angular_velocity, const Vec3 &linear_acceleration)
                : timestamp_(timestamp), angular_velocity_(angular_velocity),
                  linear_acceleration_(linear_acceleration) {
        }

    public:
        double timestamp_;
        Vec3 angular_velocity_ = Vec3::Zero(); // rad/s
        Vec3 linear_acceleration_ = Vec3::Zero();

    public:
        IMU(double timestamp, const Vec3 &angular_velocity, const Vec3 &linear_acceleration,
            const Vec3 &bg, const Vec3 &ba, const Mat3 &angular_velocity_covariance,
            const Mat3 &linear_acceleration_covariance
        )
                : timestamp_(timestamp), angular_velocity_(angular_velocity),
                  linear_acceleration_(linear_acceleration), bg_(bg), angular_velocity_covariance_(angular_velocity_covariance),
                  ba_(ba),
                  linear_acceleration_covariance_(linear_acceleration_covariance) {
        }

        Vec3 bg_;
        Mat3 angular_velocity_covariance_ = Mat3::Zero();

        Vec3 ba_;
        Mat3 linear_acceleration_covariance_ = Mat3::Zero();
    };

    using IMUPtr = std::shared_ptr<IMU>;


}

#endif //FUSION_ROS_IMU_HPP
