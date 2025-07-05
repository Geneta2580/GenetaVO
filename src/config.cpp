#include "myslam/config.h"

namespace myslam {
bool Config::SetParameterFile(const std::string &filename) {
    if (config_ == nullptr)
        config_ = std::shared_ptr<Config>(new Config);
    
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ); // opencv以只读模式打开yaml文件
    
    // 配置文件打开失败
    if (config_->file_.isOpened() == false) {     
        std::cout << "parameter file " << filename << " does not exist." << std::endl;
        config_->file_.release();
        return false;
    }
    return true;
}

Config::~Config() { // 读取完毕（解构）时将文件关闭释放
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}
