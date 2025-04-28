#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common.h"

namespace myslam {

    // 参数配置类，从文件中获取配置信息
    class Config {
       private:
            static std::shared_ptr<Config> config_;
            cv::FileStorage file_;
        
            Config() {}  // private constructor makes a singleton
        public:
            ~Config();  // 读取完毕（解构）时将文件关闭释放，需要进行函数实现
        
            // 设置yaml config文件路径
            static bool SetParameterFile(const std::string &filename); 
        
            // Get方法获取参数值，key键值就是要访问的参数类型
            template <typename T>
            static T Get(const std::string &key) {
                return T(Config::config_->file_[key]);
            }
    };
}

#endif 