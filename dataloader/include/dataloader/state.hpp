#ifndef FUSION_ROS_STATE_HPP
#define FUSION_ROS_STATE_HPP

#include "dataloader/types.hpp"

#include <vector>
#include <iostream>
#include <ostream>

#include <fmt/format.h>

namespace Fusion {
    /**
     * 由POSE轨迹计算出来的状态, 目前有速度, 加速度
     */

    class State {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        State() = default;
        /**
         *
         * @param timestamp 时间戳
         * @param angular_velocity 角速度 rad/s
         * @param linear_velocity 线速度 m/s
         * @param linear_acceleration 线加速度 m/s^2
         */
        State(double timestamp, const Vec3 &angular_velocity, const Vec3 &linear_velocity,
              const Vec3 &linear_acceleration)
                : timestamp_(timestamp), angular_velocity_(angular_velocity), linear_velocity_(linear_velocity),
                  linear_acceleration_(linear_acceleration) {}

        double timestamp_ = 0;
        // av(t) = LOG[ R(t) * R(t-1).inv() ] / dt
        // lv(t) = [p(t) - p(t-1)] / dt
        // la(t) = [lv(t) - lv(t-1)] / dt ---------- 补第一个对齐 la(0) = la(1)
        Vec3 angular_velocity_; // 角速度
        Vec3 linear_velocity_;// 线速度
        Vec3 linear_acceleration_; // 线加速度

        friend std::ostream &operator<<(std::ostream &os, const State &s) {
            std::string write_str = fmt::format(
                    "{:.7f} {:.7f} {:.7f} {:.7f} {:.3f} {:.3f} {:.3f} {:.4f} {:.4f} {:.4f}", s.timestamp_,
                    s.angular_velocity_(0, 0), s.angular_velocity_(1, 0), s.angular_velocity_(2, 0),
                    s.linear_velocity_(0, 0), s.linear_velocity_(1, 0), s.linear_velocity_(2, 0),
                    s.linear_acceleration_(0, 0), s.linear_acceleration_(1, 0), s.linear_acceleration_(2, 0)
            );
            os << write_str;
            return os;
        }


        static bool IntegrateState(vector<double> &timestamp_list,
                                   vector<Vec3> &angular_velocity_list,
                                   vector<Vec3> &linear_velocity_list,
                                   vector<Vec3> &linear_acceleration_list,
                                   vector<State> &state_list) {
            int n = static_cast<int>(timestamp_list.size());

            if (timestamp_list.empty() ||
                (timestamp_list.size() != angular_velocity_list.size()) ||
                (timestamp_list.size() != linear_acceleration_list.size()) ||
                (timestamp_list.size() != linear_velocity_list.size())) {

                return false;
            }


            state_list.reserve(n);
            for (int i = 0; i < n; i++) {
                state_list.emplace_back( timestamp_list[i], angular_velocity_list[i], linear_velocity_list[i],
                                      linear_acceleration_list[i]);
            }
            timestamp_list.clear();
            angular_velocity_list.clear();
            linear_velocity_list.clear();
            linear_acceleration_list.clear();

            return true;
        }

    };

}

#endif //FUSION_ROS_STATE_HPP
