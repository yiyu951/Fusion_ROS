#include "dataloader/dataloader.hpp"

namespace Fusion {
    DataLoader::DataLoader(const std::string &path, const std::string &base_topic)
            : Node(base_topic + "_data_loader") {
//        logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger("dataloader"));

        read_path_ = path;
        base_topic_ = base_topic;

        std::ifstream f_read{path};
        while (!f_read.eof()) {
            std::string line;
            std::getline(f_read, line);
            if (line.empty()) {
                continue;
            }
            if (line[0] == '#') {
                // 以#开头的是注释
                continue;
            }
            // load data from line
            std::stringstream ss;
            ss << line;
            std::string data_type;
            ss >> data_type;
            if (data_type == "IMU") {
                double time; // 时间戳 s
                double angular_velocity_x, angular_velocity_y, angular_velocity_z; // 角速度 rad/s
                double linear_acceleration_x, linear_acceleration_y, linear_acceleration_z; // 线加速度 m/s^2
                ss >> time >>
                   angular_velocity_x >> angular_velocity_y >> angular_velocity_z >>
                   linear_acceleration_x >> linear_acceleration_y >> linear_acceleration_z;
                IMUPtr ptr = std::make_shared<IMU>(time,
                                                   Vec3(angular_velocity_x, angular_velocity_y, angular_velocity_z) / 180. * M_PI,
                                                   Vec3(linear_acceleration_x, linear_acceleration_y,
                                                        linear_acceleration_z));
                imu_list_.emplace_back(ptr);
            } else if (data_type == "POSE") {
                double time; // 图像采集时间 / s
                double x, y, z; // t / m
                double angleAxis_x, angleAxis_y, angleAxis_z; // 旋转向量 / rad
                double total_time; // 图像位姿解算出的时间 === 发布数据的时间 / s
                ss >> time >> x >> y >> z >> angleAxis_x >> angleAxis_y >> angleAxis_z >> total_time;
                SO3 R = SO3::exp(Vec3(angleAxis_x, angleAxis_y, angleAxis_z));
                Vec3 t;
                t << x, y, z;

                POSEPtr ptr = std::make_shared<POSE>(time, SE3{R, t});
                pose_list_.emplace_back(ptr);
            } else if (data_type == "ODOM") {
                double time, speed;
                ss >> time >> speed;
                ODOMPtr ptr = std::make_shared<ODOM>(time, speed);
                odom_list_.emplace_back(ptr);
            }
        }

        std::sort(imu_list_.begin(), imu_list_.end(), [this](const IMUPtr &a, const IMUPtr &b) -> bool {
            return a->timestamp_ < b->timestamp_;
        });
        std::sort(pose_list_.begin(), pose_list_.end(), [this](const POSEPtr &a, const POSEPtr &b) -> bool {
            return a->timestamp_ < b->timestamp_;
        });
        std::sort(odom_list_.begin(), odom_list_.end(), [this](const ODOMPtr &a, const ODOMPtr &b) -> bool {
            return a->timestamp_ < b->timestamp_;
        });
        if (!imu_list_.empty()) {
            if (base_time_stamp_ == 0) {
                base_time_stamp_ = imu_list_[0]->timestamp_;
            } else {
                base_time_stamp_ = std::min(base_time_stamp_, imu_list_[0]->timestamp_);
            }
        }
        if (!pose_list_.empty()) {
            base_time_stamp_ = std::min(base_time_stamp_, pose_list_[0]->timestamp_);
        }
        if (!odom_list_.empty()) {
            base_time_stamp_ = std::min(base_time_stamp_, odom_list_[0]->timestamp_);
        }

        RCLCPP_INFO(this->get_logger(), "Data load finish! [IMU: %zu], [POSE: %zu], [ODOM: %zu], start timestamp=%lf",
                    imu_list_.size(), pose_list_.size(), odom_list_.size(), base_time_stamp_);

        initPublisher();
        initThread();
    }

    void DataLoader::initPublisher() {
        publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(base_topic_ + "/imu", 15);
        publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(base_topic_ + "/pose", 10);
        publisher_odom_ = this->create_publisher<fusion_interfaces::msg::Odom>(base_topic_ + "/odom", 10);
    }

    void DataLoader::initThread() {
        using namespace std::chrono_literals;
        base_time_point_ = std::chrono::steady_clock::now();
        imu_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                if (imu_index_ >= imu_list_.size()) {
                    break;
                }

                auto now = std::chrono::steady_clock::now();
                double timestamp = std::chrono::duration<double>(now - base_time_point_).count();
                if (timestamp >= (imu_list_[imu_index_]->timestamp_ - base_time_stamp_) ) {
                    auto msg = convertIMU2ROS(imu_list_[imu_index_]);
                    publisher_imu_->publish(msg);
                    imu_index_++;
                } else {
                    std::this_thread::sleep_for(1ms);
                }
            }

            RCLCPP_INFO(this->get_logger(), "IMU publishing finish!");
        });
        pose_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                if ( pose_index_ >= pose_list_.size()) {
                    break;
                }

                auto now = std::chrono::steady_clock::now();
                double timestamp = std::chrono::duration<double>(now - base_time_point_).count();
                if (timestamp >= pose_list_[pose_index_]->timestamp_ - base_time_stamp_) {
                    auto msg = convertPOSE2ROS(pose_list_[pose_index_]);
                    publisher_pose_->publish(msg);
                    pose_index_++;
                } else {
                    std::this_thread::sleep_for(5ms);
                }
            }

            RCLCPP_INFO(this->get_logger(), "POSE publishing finish!");
        });
        odom_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                if (odom_index_ >= odom_list_.size()) {
                    break;
                }

                auto now = std::chrono::steady_clock::now();
                double timestamp = std::chrono::duration<double>(now - base_time_point_).count();
                if (timestamp >= odom_list_[odom_index_]->timestamp_ - base_time_stamp_) {
                    auto msg = convertODOM2ROS(odom_list_[odom_index_]);
                    publisher_odom_->publish(msg);
                    odom_index_++;
                } else {
                    std::this_thread::sleep_for(2ms);
                }
            }

            RCLCPP_INFO(this->get_logger(), "ODOM publishing finish!");
        });


    }

    DataLoader::~DataLoader() {
        if (imu_thread_.joinable()) {
            imu_thread_.join();
        }
        if (pose_thread_.joinable()) {
            pose_thread_.join();
        }
        if (odom_thread_.joinable()) {
            odom_thread_.join();
        }

        RCLCPP_ERROR(this->get_logger(), "Dataloader Finish!");

        while (rclcpp::ok())
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10s);
        }
    }

    sensor_msgs::msg::Imu DataLoader::convertIMU2ROS(IMUPtr ptr) {
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = "world";
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ptr->timestamp_ * 1e9));

        msg.angular_velocity.x = ptr->angular_velocity_(0, 0);
        msg.angular_velocity.y = ptr->angular_velocity_(1, 0);
        msg.angular_velocity.z = ptr->angular_velocity_(2, 0);

        msg.linear_acceleration.x = ptr->linear_acceleration_(0, 0);
        msg.linear_acceleration.y = ptr->linear_acceleration_(1, 0);
        msg.linear_acceleration.z = ptr->linear_acceleration_(2, 0);

        return msg;
    }

    IMUPtr DataLoader::getIMUfromROS(const sensor_msgs::msg::Imu &) {
        return {};
    }

    geometry_msgs::msg::PoseStamped DataLoader::convertPOSE2ROS(POSEPtr ptr) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ptr->timestamp_ * 1e9));

        msg.pose.position.x = ptr->pose_.translation()(0, 0);
        msg.pose.position.y = ptr->pose_.translation()(1, 0);
        msg.pose.position.z = ptr->pose_.translation()(2, 0);

        msg.pose.orientation.x = ptr->pose_.unit_quaternion().x();
        msg.pose.orientation.y = ptr->pose_.unit_quaternion().y();
        msg.pose.orientation.z = ptr->pose_.unit_quaternion().z();
        msg.pose.orientation.w = ptr->pose_.unit_quaternion().w();

        return msg;
    }

    POSEPtr DataLoader::getPOSEfromROS(const geometry_msgs::msg::PoseStamped &) {
        return {};
    }

    fusion_interfaces::msg::Odom DataLoader::convertODOM2ROS(ODOMPtr ptr) {
        fusion_interfaces::msg::Odom msg;
        msg.header.frame_id = "world";
        msg.header.stamp = rclcpp::Time(static_cast<int64_t>(ptr->timestamp_ * 1e9));

        msg.speed = ptr->speed_;

        return msg;
    }

    ODOMPtr DataLoader::getODOMfromROS(const fusion_interfaces::msg::Odom &) {
        return {};
    }

    void DataLoader::AnalysisData() {
        int n = static_cast<int>(pose_list_.size());
        RCLCPP_INFO(this->get_logger(), "AnalysisData Size = %d", n);
        vector<double> timestamp_list(n - 1, 0);
        // av(t) = LOG[ R(t) * R(t-1).inv() ] / dt
        vector<Vec3> angular_velocity_list(n - 1, Vec3(0, 0, 0)); // 角速度
        // lv(t) = [p(t) - p(t-1)] / dt
        vector<Vec3> linear_velocity_list(n - 1, Vec3(0, 0, 0));// 线速度
        // la(t) = [lv(t) - lv(t-1)] / dt ---------- 补第一个对齐 la(0) = la(1)
        vector<Vec3> linear_acceleration_list(n - 1, Vec3(0, 0, 0)); // 线加速度
        for (int i = 1; i < n; i++) {
            double dt = pose_list_[i]->timestamp_ - pose_list_[i - 1]->timestamp_;
            /// time, av, lv 都对原数组少了一位, 所以用 i-1
            // timestamp
            timestamp_list[i - 1] = pose_list_[i]->timestamp_;
            // angular_velocity
            Mat3 ri = pose_list_[i]->pose_.so3().matrix();
            Mat3 ri_1 = pose_list_[i - 1]->pose_.so3().matrix();
            angular_velocity_list[i - 1] = SO3(
                    (ri * ri_1.inverse())
            ).log() / dt;

            // linear_velocity
            Vec3 pi = pose_list_[i]->pose_.translation();
            Vec3 pi_1 = pose_list_[i - 1]->pose_.translation();
            linear_velocity_list[i - 1] = (pi - pi_1) / dt;
        }

        for (int i = 1; i < n - 1; i++) {
            double dt = timestamp_list[i] - timestamp_list[i - 1];
            /// la 都对lv没有少, 所以用i
            // linear_acceleration
            Vec3 vi = linear_velocity_list[i];
            Vec3 vi_1 = linear_velocity_list[i - 1];
            linear_acceleration_list[i] = (vi - vi_1) / dt;
        }
        linear_acceleration_list[0] = linear_acceleration_list[1];


        State::IntegrateState(timestamp_list, angular_velocity_list, linear_velocity_list,
                                          linear_acceleration_list,
                                          analysis_result_);
//        RCLCPP_INFO(this->get_logger(), "Analysis Result = %d Size = %zu", succ, analysis_result_.size());
    }

    bool DataLoader::WriteAnalysisResult(std::ofstream &out) {
        if (!out.is_open()) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Analysis Result Size = %zu", analysis_result_.size());
        for (const State &s: analysis_result_) {
            out << s << std::endl;
        }
        RCLCPP_INFO(this->get_logger(), "Write Analysis Result Success");
        out.close();
        return true;
    }
} // Fusion