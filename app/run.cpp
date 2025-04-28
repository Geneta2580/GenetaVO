#include "myslam/common.h"
#include "myslam/dataset.h"
#include <opencv2/highgui.hpp>
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/config.h"
#include "myslam/backend.h"

namespace fs = std::filesystem;

int main()
{
    try {
        const std::string config_file_path_ = "/home/geneta/Project/geneta_slam/config/config.yaml"; // 设置配置文件路径
        if (myslam::Config::SetParameterFile(config_file_path_) == false) { // 设置配置文件
            return false;
        }

        // 初始化图像加载器（假设图像路径为/home/geneta/Project/04/image_0/000000.png）
        const std::string dataset_path = myslam::Config::Get<std::string>("dataset_dir");        

        myslam::Dataset dataset(dataset_path);
        myslam::Frontend frontend_; // 创建前端，由于viewer等已在前端的构造函数中初始化，所以不用在这里初始化

        // 统计文件夹下的图片数量
        int count = 0;
        for (const auto& entry : fs::directory_iterator(dataset_path + "/image_0")) {
            if (entry.is_regular_file()) {  
                count++;
                // std::cout << "running" << std::endl;
            }
        }

        // 连续加载图像序列
        while(true) {
            myslam::Frame::Ptr frame = dataset.loadNextFrame(); // 加载一帧图像（计数值id会自动+1）

            // 显示图像基本信息，读入时进行检查
            // std::cout << "Processing left frame: " << frame->id_     // 使用智能指针时使用->
            //           << " | Size: " << frame->left_img_.cols << "x" << frame->left_img_.rows
            //           << " | Type: " << frame->left_img_.type() << std::endl;

            // std::cout << "Processing right frame: " << frame->id_
            // << " | Size: " << frame->right_img_.cols << "x" << frame->right_img_.rows
            // << " | Type: " << frame->right_img_.type() << std::endl;

            // 此处添加SLAM处理逻辑（示例显示图像）
            auto t1 = std::chrono::steady_clock::now(); // 算法计时
            frontend_.Calculate(frame); // 这里参数传进去经过操作是会改变的
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << "GenetaSLAM Frontend cost time: " << time_used.count() << " seconds." << std::endl;

            // cv::imshow("SLAM Left Frame Preview", frame->left_img_); // 注意imshow多线程竞争问题
            // cv::imshow("SLAM Right Frame Preview", frame->right_img_);
            // cv::waitKey(1);          // 等待按键输入
            // std::cout << "running" << std::endl;

            // 检查是否到达序列末尾
            if(frame->id_ == (count-1)) {  
                std::cout << "Reached end of image sequence at frame: " 
                            << frame->id_ << std::endl;
                frontend_.Stop();
                break;
            }
        }

    } 
    catch(const std::exception& e) {
        std::cerr << "[FATAL ERROR] " << e.what() << std::endl; // 异常处理机制
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}