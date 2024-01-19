#include "eskf/eskf_node.hpp"
#include <rclcpp/logging.hpp>

namespace Fusion {

    ESKFNode::ESKFNode(const std::string &node_name, const std::string &base_topic)
            : Node(node_name), base_topic_(base_topic) {

        std::string logging_level = declare_parameter<std::string>("log_level", "INFO");
        if (logging_level == "DEBUG") {
            auto ret = rcutils_logging_set_logger_level(
                    get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
                rcutils_reset_error();
            }
        }

        initSubscription();

        // eskf option
        eskf_option_.imu_dt = 0.01; //

        eskf_option_.gyro_var_ = 7.8018249029542300e-04;      // 陀螺测量标准差
        eskf_option_.acce_var_ = 4.4845426051592311e-03;      // 加计测量标准差
        eskf_option_.bias_gyro_var_ = 6.2986848574373341e-06; // 陀螺零偏游走标准差
        eskf_option_.bias_acce_var_ = 2.2244092728596087e-04; // 加计零偏游走标准差

        eskf_option_.trans_noise = 0.01;
        eskf_option_.angle_noise = 0.001;

        eskf_option_.update_bias_gyro_ = true; // 是否更新陀螺bias
        eskf_option_.update_bias_acce_ = true; // 是否更新加计bias
    }


    void ESKFNode::initSubscription() {
        using std::placeholders::_1;
        subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(base_topic_ + "/imu", 10,
                                                                             std::bind(&ESKFNode::imu_callback, this,
                                                                                       _1));
        subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(base_topic_ + "/pose", 10,
                                                                                        std::bind(
                                                                                                &ESKFNode::pose_callback,
                                                                                                this, _1));
        subscription_odom_ = this->create_subscription<fusion_interfaces::msg::Odom>(base_topic_ + "/odom", 10,
                                                                                     std::bind(&ESKFNode::odom_callback,
                                                                                               this, _1));
    }


    IMUPtr ESKFNode::getIMUfromROS(const sensor_msgs::msg::Imu &msg) {
        double timestamp;
        Vec3 angular_velocity, linear_acceleration;

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
        angular_velocity << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
        linear_acceleration << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;

        return std::make_shared<IMU>(timestamp, angular_velocity, linear_acceleration);
    }

    POSEPtr ESKFNode::getPOSEfromROS(const geometry_msgs::msg::PoseStamped &msg) {
        double timestamp;
        Vec3 translation;
        Eigen::Quaterniond quaternion;
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
        translation << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        quaternion.x() = msg.pose.orientation.x;
        quaternion.y() = msg.pose.orientation.y;
        quaternion.z() = msg.pose.orientation.z;
        quaternion.w() = msg.pose.orientation.w;

        SE3 pose{quaternion, translation};

        return std::make_shared<POSE>(timestamp, pose);
    }

    ODOMPtr ESKFNode::getODOMfromROS(const fusion_interfaces::msg::Odom &msg) {
        double timestamp, speed;
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
        speed = msg.speed;

        return std::make_shared<ODOM>(timestamp, speed);
    }

    void ESKFNode::imu_callback(const sensor_msgs::msg::Imu &msg) {
        IMUPtr ptr = getIMUfromROS(msg);
        RCLCPP_DEBUG(this->get_logger(), "Get IMU, timestamp = %.7lf", ptr->timestamp_);
        // IMU初始化成功
        if (imuInit_.GetInitSuccess()) {
            // 第一个POSE还没收到, 跳过
            if (!first_pose_) {
                return;
            }

            // 静止时, 不使用IMU的数据, 即不进行ESKF的预测, 只更新一下ESKF内部的时间
            if (is_static_) {
                RCLCPP_DEBUG(this->get_logger(), "Static, don't use IMU");
                eskf_->UpdateTime(ptr->timestamp_);
                return;
            }
            State state;
            bool succ = eskf_->Predict(ptr, state);
            if (!succ) {
                return;
            }

            RCLCPP_DEBUG_STREAM(this->get_logger(), "State: " << state);

//            if (!analysis_result_.empty()) {
//                double dt = state.timestamp_ - analysis_result_.back().timestamp_;
//                state.linear_acceleration_ = (state.linear_velocity_ - analysis_result_.back().linear_velocity_) / dt;
//            }
//            if (analysis_result_.size() == 1) {
//                analysis_result_[0].linear_acceleration_ = state.linear_acceleration_;
//            }
            analysis_result_.emplace_back(state);
            eskf_state_list_.emplace_back(eskf_->GetESKFState());

        } else if (is_static_) {
            // 否则等待静止, 去初始化IMU
            if (imuInit_.AddIMU(ptr)) {
                RCLCPP_DEBUG_STREAM(this->get_logger(),
                                    "IMU Init Success: bg=" << imuInit_.GetInitBg().transpose()
                                                            << ", ba=" << imuInit_.GetInitBa().transpose()
                                                            << ", g=" << imuInit_.GetGravity());
                // 初始化成功, 使用参数来赋值eskf
                eskf_option_.gyro_var_ = sqrt(imuInit_.GetCovGyro()[0]);      // 陀螺测量标准差
                eskf_option_.acce_var_ = sqrt(imuInit_.GetCovAcce()[0]);      // 加计测量标准差

                eskf_ = std::make_unique<ESKF>(eskf_option_, imuInit_.GetInitBg(), imuInit_.GetInitBa(),
                                               imuInit_.GetGravity());

            } else {
                RCLCPP_DEBUG(this->get_logger(), "正在静止初始化IMU");
            }

        } else {
            RCLCPP_DEBUG(this->get_logger(), "IMU未初始化, 未静止");
        }
    }

    void ESKFNode::pose_callback(const geometry_msgs::msg::PoseStamped &msg) {
        //
        POSEPtr ptr = getPOSEfromROS(msg);
        RCLCPP_DEBUG(this->get_logger(), "Get Pose, timestamp = %.7lf", ptr->timestamp_);
        if (!imuInit_.GetInitSuccess()) {
            RCLCPP_DEBUG(this->get_logger(), "Get Pose, Waiting IMU Init");
            return;
        }
        if (!first_pose_) {
            eskf_->SetPose(ptr);
            first_pose_ = true;
            return;
        }

        eskf_->Observe(ptr);
    }

    void ESKFNode::odom_callback(const fusion_interfaces::msg::Odom &msg) {
        ODOMPtr ptr = getODOMfromROS(msg);
        RCLCPP_DEBUG(this->get_logger(), "Get Odom, timestamp = %.7lf speed = %lf", ptr->timestamp_, ptr->speed_);

        if (ptr->speed_ < min_speed_) {
            is_static_ = true;
        } else {
            is_static_ = false;
        }
    }


    void ESKFNode::WriteAnalysisData(std::ofstream &out) {
        if (!out.is_open()) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Analysis Result Size = %zu", analysis_result_.size());
        for (const State &s: analysis_result_) {
            out << s << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "Write Analysis Result Success");
        out.close();
        return;
    }

    ESKFNode::~ESKFNode() {
        RCLCPP_ERROR(this->get_logger(), "ESKF Node will destroy! Writing AnalysisData");
        std::ofstream out{"/home/zhonglingjun/GXT/Fusion_ROS/src/dataloader/data/AnalysisDataIMU.txt"};
        WriteAnalysisData(out);
        RCLCPP_ERROR(this->get_logger(), "ESKF Node destroied!");
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Fusion::ESKFNode> node = std::make_shared<Fusion::ESKFNode>(
            "eskf_test", "test_analysis_data"
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}