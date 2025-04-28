#pragma once
#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "myslam/common.h"
#include "myslam/frame.h"
#include <opencv2/imgcodecs.hpp> // 读取图像用
#include <opencv2/imgproc.hpp>
#include <boost/format.hpp> // 规定路径格式用
#include <sys/stat.h> // 用于文件存在性检查
#include <glog/logging.h> // 用于输出日志

namespace myslam {

// 数据集类，用以实现图像数据队列的读取
class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;
        explicit Dataset(const std::string& dataset_path);
        
        // 加载下一帧图像
        Frame::Ptr loadNextFrame(); // 因为是两张图像，所以用frame的智能指针
    
    private:
        int current_index_ = 0;
        std::string dataset_path_; // 注意下划线位置要和.cpp中声明:后的变量一致

    };
}

#endif
