#ifndef FUSION_ROS_IMU_INIT_HPP
#define FUSION_ROS_IMU_INIT_HPP

#include "eskf/eskf.hpp"

#include <deque>
#include <vector>
#include <atomic>

#include <dataloader/imu.hpp>

namespace Fusion {

    class IMUInit {
    public:
        struct Option {
            Option() {}

            double init_time_seconds_ = 2.0;            // 静止时间
            int init_imu_queue_max_size_ = 2000;        // 初始化IMU队列最大长度
            double max_static_gyro_var = 0.5;           // 静态下陀螺测量方差
            double max_static_acce_var = 0.5;           // 静态下加计测量方差
            double gravity_norm_ = 9.81;                // 重力大小
        };

        IMUInit() = default;

        bool AddIMU(IMUPtr imu);

        bool GetInitSuccess() { return imu_init_success_; }

        Vec3 GetCovGyro() const { return angular_velocity_covariance_; }

        Vec3 GetCovAcce() const { return linear_acceleration_covariance_; }

        Vec3 GetInitBg() const { return init_bg_; }

        Vec3 GetInitBa() const { return init_ba_; }

        Vec3 GetGravity() const { return g_; }

    private:
        bool TryInit();

        Option option_;
        std::atomic_bool imu_init_success_{false};
        std::atomic_bool is_static_{false};
        std::deque<IMUPtr> imu_deque_;

        double current_time_ = 0.0;
        double init_start_time_ = 0.0;

        Vec3 init_bg_ = Vec3::Zero(); // 陀螺初始零偏
        Vec3 init_ba_ = Vec3::Zero(); // 加计初始零偏
        Vec3 g_ = Vec3::Zero();// 重力
        Vec3 angular_velocity_covariance_ = Vec3::Zero(); // 陀螺测量噪声协方差（初始化时评估）
        Vec3 linear_acceleration_covariance_ = Vec3::Zero(); // 加计测量噪声协方差（初始化时评估）

    };
}


#endif //FUSION_ROS_IMU_INIT_HPP
