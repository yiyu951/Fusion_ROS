#include "dataloader/dataloader.hpp"
#include <fstream>
//#include <gtest/gtest.h>


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    std::shared_ptr<Fusion::DataLoader> dataLoader = std::make_shared<Fusion::DataLoader>(
                "/home/zhonglingjun/GXT/Fusion_test/data/output_file_sort.txt",
//            "/home/zhonglingjun/GXT/Fusion_test/data/euroc_data_fusion_V2_01_easy.txt",
            "test_analysis_data");
    RCLCPP_INFO(dataLoader->get_logger(), "AnalysisData");
    dataLoader->AnalysisData();
    std::ofstream out{"/home/zhonglingjun/GXT/Fusion_ROS/src/dataloader/data/write/AnalysisDataPOSE.txt"};
    dataLoader->WriteAnalysisResult(out);
    RCLCPP_INFO(dataLoader->get_logger(), "WriteAnalysisResult");

    rclcpp::spin(dataLoader);
    rclcpp::shutdown();

    return 0;
}