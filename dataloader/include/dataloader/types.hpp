#ifndef FUSION_ROS_TYPES_HPP
#define FUSION_ROS_TYPES_HPP

#include <eigen3/Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

namespace Fusion{
    /// 类型定义
    using SO3 = Sophus::SO3<double>;                    // 旋转变量类型
    using SE3 = Sophus::SE3<double>;
    using Vec3 = Eigen::Vector3d;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;
    using Vec18 = Eigen::Matrix<double, 18, 1>;        // 18维向量类型
    using Mat3 = Eigen::Matrix<double, 3, 3>;          // 3x3矩阵类型
    using Mat18 = Eigen::Matrix<double, 18, 18>;       // 18维方差类型

    using std::vector;
}

#endif //FUSION_ROS_TYPES_HPP
