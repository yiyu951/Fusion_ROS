#include "eskf/imu_init.hpp"

#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <cmath>

template<typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C &data, D &mean, D &cov_diag, Getter &&getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D &sum, const auto &data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D &sum, const auto &data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}

bool Fusion::IMUInit::AddIMU(Fusion::IMUPtr imu) {
    if (imu_init_success_) {
        return true;
    }

    if (imu_deque_.empty()) {
        init_start_time_ = imu->timestamp_;
    }

    imu_deque_.emplace_back(imu);
    double deque_time = imu->timestamp_ - init_start_time_;
    if (deque_time > option_.init_time_seconds_) {
        // 尝试初始化逻辑
        TryInit();
    }

    // 维持初始化队列长度
    while (static_cast<int>(imu_deque_.size()) > option_.init_imu_queue_max_size_) {
        imu_deque_.pop_front();
    }

    current_time_ = imu->timestamp_;
    return imu_init_success_;
}

bool Fusion::IMUInit::TryInit() {
    if (imu_deque_.size() < 10) {
        LOG(ERROR) << "IMU标定队列较短, size = " << imu_deque_.size();
        return false;
    }
    // 计算均值和方差
    Vec3 mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(imu_deque_, mean_gyro, angular_velocity_covariance_,
                          [](const IMUPtr &imu) { return imu->angular_velocity_; });
    ComputeMeanAndCovDiag(imu_deque_, mean_acce, linear_acceleration_covariance_,
                          [this](const IMUPtr &imu) { return imu->linear_acceleration_; });
// 以acce均值为方向，取9.8长度为重力
    g_ = -mean_acce / mean_acce.norm() * option_.gravity_norm_;

    // 重新计算加计的协方差
    ComputeMeanAndCovDiag(imu_deque_, mean_acce, linear_acceleration_covariance_,
                                [this](const IMUPtr &imu)
                                { return imu->linear_acceleration_ + g_; });

    // 检查IMU噪声
    if (angular_velocity_covariance_.norm() > option_.max_static_gyro_var)
    {
        LOG(ERROR) << "陀螺仪测量噪声太大" << angular_velocity_covariance_.norm() << " > " << option_.max_static_gyro_var;
        return false;
    }

    if (linear_acceleration_covariance_.norm() > option_.max_static_acce_var)
    {
        LOG(ERROR) << "加计测量噪声太大" << linear_acceleration_covariance_.norm() << " > " << option_.max_static_acce_var;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro sq = " << angular_velocity_covariance_.transpose()
              << ", acce sq = " << linear_acceleration_covariance_.transpose() << ", grav = " << g_.transpose()
              << ", norm: " << g_.norm();
    LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
    imu_init_success_ = true;

    imu_deque_.clear();
    return true;


}
