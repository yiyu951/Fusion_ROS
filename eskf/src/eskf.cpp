#include "eskf/eskf.hpp"
#include <fmt/format.h>

namespace Fusion {

    void ESKF::BuildNoise(Option option) {
        // 观测噪声矩阵 POSE
        V_.diagonal() << option.angle_noise, option.angle_noise, option.angle_noise,
                option.trans_noise, option.trans_noise, option.trans_noise;

        // 预测噪声矩阵 imu
        double ev = option.acce_var_;
        double et = option.gyro_var_;
        double eg = option.bias_gyro_var_;
        double ea = option.bias_acce_var_;

        double ev2 = ev; // * ev;
        double et2 = et; // * et;
        double eg2 = eg; // * eg;
        double ea2 = ea; // * ea;

        Q_.diagonal() << 0, 0, 0,
                ev2, ev2, ev2,
                et2, et2, et2,
                eg2, eg2, eg2,
                ea2, ea2, ea2,
                0, 0, 0;
    }

    void ESKF::UpdateTime(double timestamp) {
        current_time_ = timestamp;
    }

    void ESKF::SetPose(const POSEPtr &pose) {
        R_ = pose->pose_.so3();
        p_ = pose->pose_.translation();
    }


    bool ESKF::Predict(IMUPtr imu, State& state) {
        double dt = imu->timestamp_ - current_time_;

        LOG(INFO) << fmt::format("dt = {:.2f}ms ,{:.6f} - {:.6f}", dt * 1e3, imu->timestamp_, current_time_);

        if (dt > (5 * option_.imu_dt) || dt < 0) {
            LOG(ERROR) << "dt > 5*imu_dt || dt < 0";
            return false;
        }
        //
        state.timestamp_ = imu->timestamp_;
        state.angular_velocity_ = imu->angular_velocity_ -bg_;
        state.linear_velocity_ = v_;
        state.linear_acceleration_ = imu->linear_acceleration_ + g_ - ba_;

        std::lock_guard<std::mutex> l{eskfMutex};

        Vec3 new_p = p_ + v_ * dt + 0.5 * (R_ * (imu->linear_acceleration_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
        Vec3 new_v = v_ + R_ * (imu->linear_acceleration_ - ba_) * dt + g_ * dt;
        SO3 new_R = R_ * SO3::exp((imu->angular_velocity_ - bg_) * dt);


        R_ = new_R;
        v_ = new_v;
        p_ = new_p;

        // error state 递推
        // 计算运动过程雅可比矩阵 F，见(3.47)
        // F实际上是稀疏矩阵，也可以不用矩阵形式进行相乘而是写成散装形式，这里为了教学方便，使用矩阵形式
        Mat18 F = Mat18::Identity();                                                // 主对角线
        F.template block<3, 3>(0, 3) = Mat3::Identity() * dt;                        // p 对 v
        F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu->linear_acceleration_ - ba_) * dt; // v对theta
        F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                            // v 对 ba
        F.template block<3, 3>(3, 15) = Mat3::Identity() * dt;                       // v 对 g
        F.template block<3, 3>(6, 6) = SO3::exp(-(imu->angular_velocity_ - bg_) * dt).matrix();    // theta 对 theta
        F.template block<3, 3>(6, 9) = -Mat3::Identity() * dt;                       // theta 对 bg

        cov_ = F * cov_.eval() * F.transpose() + Q_;
        current_time_ = imu->timestamp_;

        return true;
    }

    void ESKF::Observe(POSEPtr pose) {
        // 观测状态变量中的p, R，H为6x18，其余为零
        Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
        H.template block<3, 3>(0, 0) = Mat3::Identity(); // P部分
        H.template block<3, 3>(3, 6) = Mat3::Identity(); // R部分（3.66)

        std::lock_guard<std::mutex> l{eskfMutex};

        Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V_).inverse();
        // 更新x和cov
        Vec6 innov = Vec6::Zero();
        innov.template head<3>() = (pose->pose_.translation() - p_);         // 平移部分
        innov.template tail<3>() = (R_.inverse() * pose->pose_.so3()).log(); // 旋转部分(3.67)

        dx_ = K * innov;
        cov_ = (Mat18::Identity() - K * H) * cov_;
        UpdateAndReset();
        // 更新时间
        current_time_ = pose->timestamp_;
    }

    /**
     * @brief   根据观测的POSE, 计算dx, 更新名义变量: p, v, r, bg, ba, g
     */
    void ESKF::UpdateAndReset() {
        p_ += dx_.template block<3, 1>(0, 0);
        v_ += dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        if (option_.update_bias_gyro_) {
            bg_ += dx_.template block<3, 1>(9, 0);
        }

        if (option_.update_bias_acce_) {
            ba_ += dx_.template block<3, 1>(12, 0);
        }

        g_ += dx_.template block<3, 1>(15, 0);


        ProjectCov();
        dx_.setZero();
    }

    void ESKF::ProjectCov() {
        Mat18 J = Mat18::Identity();
        J.template block<3, 3>(6, 6) = Mat3::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

    ESKFStatePtr ESKF::GetESKFState() {
        std::lock_guard<std::mutex> l{eskfMutex};
        return std::make_shared<ESKFState>(current_time_, p_, v_, R_, bg_, ba_, g_);
    }


}