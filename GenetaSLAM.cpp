#include "myslam/GenetaSLAM.h"

namespace myslam {

    GenetaSLAM::GenetaSLAM(const std::string &config_path)
    : config_file_path_(config_path) {}

    bool GenetaSLAM::Init() {
        if (Config::SetParameterFile(config_file_path_) == false) { // 设置配置文件
            return false;
        }

        dataset_ = std::make_shared<Dataset>(Config::Get<std::string>("dataset_dir")); // 进行数据集类的初始化
        frontend_ = std::make_shared<Frontend>(); // 创建前端，由于viewer等已在前端的构造函数中初始化，所以不用在这里初始化
        backend_ = std::make_shared<Backend>(); // 创建后端，挂起等待关键帧更新
        viewer_ = std::make_shared<Viewer>(); // 创建可视化界面，循环渲染输入的帧和地图点

        frontend_->SetBackend(backend_);
        frontend_->SetViewer(viewer_);


        return true;
    }

    bool GenetaSLAM::Step() {
        Frame::Ptr new_frame = dataset_->loadNextFrame(); // 加载下一帧
        if (new_frame == nullptr) return false; // 下一帧为空，即到文件末尾

        // auto t1 = std::chrono::steady_clock::now(); // 算法计时
        frontend_->Calculate(new_frame); // 前端计算新帧
        // auto t2 = std::chrono::steady_clock::now();
        // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "GenetaSLAM cost time: " << time_used.count() << " seconds." << std::endl;
        return true;
    }

    void GenetaSLAM::Run() {
        while (1) {
            std::cout << "GenetaSLAM is running" << std::endl;
            if (Step() == false) {
                break;
            }
        }
    
        backend_->Stop();
        viewer_->Close();
    
        std::cout << "GenetaSLAM exit" << std::endl;
    }
}