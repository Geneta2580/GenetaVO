#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common.h"
#include "myslam/mappoint.h"

namespace myslam {

    struct Frame;    // 前向声明，头文件相互引用时加入
    struct MapPoint;

    //特征点类，用以存储每帧图像提取的特征点
    struct Feature {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Feature> Ptr;
        
            std::weak_ptr<Frame> frame_;         // 持有该feature的frame
            cv::KeyPoint position_;              // 2D提取位置,注意这里成员变量不能封装成vector,但可以先整体封装成vector然后通过循环存储到对应类型的vector
            std::weak_ptr<MapPoint> map_point_;  // 关联地图点
        
            bool is_outlier_ = false;       // 是否为异常点
            bool is_on_left_image_ = true;  // 标识是否提在左图，false为右图
     
        public:
            Feature() {}
        
            Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp) // 参数化构造函数
                : frame_(frame), position_(kp) {}
    };
}

#endif