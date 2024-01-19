#ifndef FUSION_ROS_DATALOADER_HPP
#define FUSION_ROS_DATALOADER_HPP

#include "dataloader/types.hpp"
#include "dataloader/imu.hpp"
#include "dataloader/pose.hpp"
#include "dataloader/odom.hpp"
#include "dataloader/state.hpp"

#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <fstream>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "fusion_interfaces/msg/odom.hpp"

namespace Fusion {


    class DataLoader : public rclcpp::Node {
    public:

        /**
         * @brief 读取文件, 分析数据, 发布话题
         * @param path: 文件路径
         */
        explicit DataLoader(const std::string &path, const std::string &base_topic);

        ~DataLoader() override;

    public:
        /**
         * @brief 分析数据, 数据中有IMU、POSE、ODOM, 计算POSE轨迹的速度、加速度等指标。
         */
        void AnalysisData();

        bool WriteAnalysisResult(std::ofstream &out);

        vector<State> GetAnalysisResult() { return analysis_result_; }

    private:
        vector<State> analysis_result_;

    private:
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> publisher_imu_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped >> publisher_pose_;
        std::shared_ptr<rclcpp::Publisher<fusion_interfaces::msg::Odom >> publisher_odom_;

        void initPublisher();

    private:
        /// 基础数据
        static sensor_msgs::msg::Imu convertIMU2ROS(IMUPtr ptr);

        static geometry_msgs::msg::PoseStamped convertPOSE2ROS(POSEPtr ptr);

        static fusion_interfaces::msg::Odom convertODOM2ROS(ODOMPtr ptr);

        static IMUPtr getIMUfromROS(const sensor_msgs::msg::Imu &);

        static POSEPtr getPOSEfromROS(const geometry_msgs::msg::PoseStamped &);

        static ODOMPtr getODOMfromROS(const fusion_interfaces::msg::Odom &);

    private:
        std::chrono::steady_clock::time_point base_time_point_;
        double base_time_stamp_;

        vector<IMUPtr> imu_list_;
        unsigned int imu_index_ = 0;
        std::thread imu_thread_;

        vector<POSEPtr> pose_list_;
        unsigned int pose_index_ = 0;
        std::thread pose_thread_;

        vector<ODOMPtr> odom_list_;
        unsigned int odom_index_ = 0;
        std::thread odom_thread_;

        void initThread();

        std::string base_topic_;
        std::string read_path_;
        std::string write_path_;
    };

} // Fusion

#endif //FUSION_ROS_DATALOADER_HPP
