#ifndef _ESKF_ESKF_HPP_
#define _ESKF_ESKF_HPP_

#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

#include <mutex>
#include <memory>


#include <dataloader/types.hpp>
#include <dataloader/imu.hpp>
#include <dataloader/pose.hpp>
#include "dataloader/state.hpp"

#include "eskf/imu_init.hpp"

#include <glog/logging.h>

namespace Fusion {

    class ESKFState {
    public:
        ESKFState(double timestamp, const Vec3 &p, const Vec3 &v, const SO3 &R,
                  const Vec3 &bg, const Vec3 &ba, const Vec3 &g)
                : timestamp_(timestamp), p_(p), v_(v), R_(R), bg_(bg), ba_(ba), g_(g) {}

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // 当前时间
        double timestamp_;
        // 名义状态
        Vec3 p_;
        Vec3 v_;
        SO3 R_;
        Vec3 bg_;
        Vec3 ba_;
        Vec3 g_;

        friend std::ostream &operator<<(std::ostream &os, const ESKFState &s) {
            std::string write_str = fmt::format(
                    "time = {:.7f} p = {:.3f} {:.3f} {:3f} v = {:.3f} {:.3f} {:.3f} q = {:.3f} {:.3f} {:.3f} {:.3f}",
                    s.timestamp_,
                    s.p_(0, 0), s.p_(1, 0), s.p_(2, 0),
                    s.v_(0, 0), s.v_(1, 0), s.v_(2, 0),
                    s.R_.unit_quaternion().x(), s.R_.unit_quaternion().y(), s.R_.unit_quaternion().z(),
                    s.R_.unit_quaternion().w()
            );
            os << write_str;
            return os;
        }

    };

    using ESKFStatePtr = std::shared_ptr<ESKFState>;

    class ESKF {

    public:
        struct Option {
            Option() = default;

            double imu_dt = 0.01; //
            // NOTE IMU噪声项都为离散时间，不需要再乘dt，可以由初始化器指定IMU噪声
            // 预测噪声，更新
            double gyro_var_ = 7.8018249029542300e-04;      // 陀螺测量标准差
            double acce_var_ = 4.4845426051592311e-03;      // 加计测量标准差
            double bias_gyro_var_ = 6.2986848574373341e-06; // 陀螺零偏游走标准差
            double bias_acce_var_ = 2.2244092728596087e-04; // 加计零偏游走标准差

            // 观测噪声, 不更新
            double trans_noise = 0.01;
            double angle_noise = 0.001;
            /// 其他配置
            bool update_bias_gyro_ = true; // 是否更新陀螺bias
            bool update_bias_acce_ = true; // 是否更新加计bias
        };


    public:
        explicit ESKF(const Option option,
                      const Vec3 &init_bg = Vec3::Zero(), const Vec3 &init_ba = Vec3::Zero(),
                      const Vec3 &gravity = Vec3::Zero()) : option_(option) {
            BuildNoise(option);
            bg_ = init_bg;
            ba_ = init_ba;
            g_ = gravity;
        }

        void BuildNoise(Option option);

        // 更新内部时间, 静止时不使用IMU
        void UpdateTime(double timestamp);

        // 预测
        bool Predict(IMUPtr imu, State& state);

        // 观测
        void Observe(POSEPtr pose);

        ESKFStatePtr GetESKFState();

    private:
        void UpdateAndReset();

        void ProjectCov();

    public:

        void SetOption(Option option) { option_ = option; }

        void SetPose(const POSEPtr &pose);

        /// Get functions
        double GetTime() const { return current_time_; }

        Vec3 GetP() const { return p_; }

        Vec3 GetV() const { return v_; }

        SO3 GetR() const { return R_; }

        SE3 GetT() const { return SE3{R_, p_}; }

        Vec3 GetBg() const { return bg_; }

        Vec3 GetBa() const { return ba_; }

        Vec3 GetG() const { return g_; }

        Vec18 GetError() const { return dx_; }

        Mat18 GetCov() const { return cov_; }

        Mat18 GetCovQ() const { return Q_; }

        Mat6 GetCovV() const { return V_; }

    private:

        // 当前时间
        double current_time_ = 0;
        // 名义状态
        Vec3 p_ = Vec3::Zero();
        Vec3 v_ = Vec3::Zero();
        SO3 R_;
        Vec3 bg_ = Vec3::Zero();
        Vec3 ba_ = Vec3::Zero();
        Vec3 g_ = Vec3::Zero();

        // 误差状态
        Vec18 dx_ = Vec18::Zero();

        // 协方差阵
        Mat18 cov_ = Mat18::Zero();

        // 噪声阵
        Mat18 Q_ = Mat18::Zero(); // 预测噪声, 更新
        Mat6 V_ = Mat6::Zero(); // 观测噪声, 不更新

        std::mutex eskfMutex;
        Option option_;
    };


}

#endif