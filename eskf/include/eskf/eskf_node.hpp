#ifndef FUSION_ROS_ESKF_NODE_HPP
#define FUSION_ROS_ESKF_NODE_HPP

#include "eskf/eskf.hpp"
#include "eskf/imu_init.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "fusion_interfaces/msg/odom.hpp"

#include "dataloader/imu.hpp"
#include "dataloader/pose.hpp"
#include "dataloader/odom.hpp"
#include "dataloader/state.hpp"

#include <string>
#include <memory>
#include <fstream>

namespace Fusion {
    /**
     * 接收IMU, POSE, ODOM数据送入ESKF
     * TODO: 添加step功能, 实现步调
     */
    class ESKFNode : public rclcpp::Node {
    public:
        explicit ESKFNode(const std::string &node_name, const std::string &base_topic);

        ~ESKFNode() override;

        void WriteAnalysisData(std::ofstream & out);

    private:
        std::string base_topic_;

        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Imu>> subscription_imu_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped >> subscription_pose_;
        std::shared_ptr<rclcpp::Subscription<fusion_interfaces::msg::Odom >> subscription_odom_;

        void initSubscription();

        void imu_callback(const sensor_msgs::msg::Imu &msg);

        void pose_callback(const geometry_msgs::msg::PoseStamped &msg);

        void odom_callback(const fusion_interfaces::msg::Odom &msg);

    private:

        static IMUPtr getIMUfromROS(const sensor_msgs::msg::Imu &);

        static POSEPtr getPOSEfromROS(const geometry_msgs::msg::PoseStamped &);

        static ODOMPtr getODOMfromROS(const fusion_interfaces::msg::Odom &);


    private:
        vector<ESKFStatePtr> eskf_state_list_;
        vector<State> analysis_result_;

        std::unique_ptr<ESKF> eskf_{nullptr};
        ESKF::Option eskf_option_;

        std::atomic_bool first_pose_{false};

        double min_speed_ = 1.0;
        std::atomic_bool is_static_{false};

        IMUInit imuInit_;
    };

}

#endif //FUSION_ROS_ESKF_NODE_HPP
