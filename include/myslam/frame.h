#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include <opencv2/opencv.hpp>
#include "myslam/common.h"
#include "myslam/feature.h"

namespace myslam {
    // 帧图像类，用于存储读取到的图像
    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;
        
        // 构造函数（包含默认参数）
        unsigned long id_ = 0;           // id of this frame 公有变量
        unsigned long keyframe_id_ = 0;  // id of key frame
        double time_stamp_;              // 时间戳，暂不使用
        bool is_keyframe_ = false;       // 是否为关键帧
        SE3 pose_;                       // Tcw 形式Pose
        std::mutex pose_mutex_;          // Pose数据锁
        cv::Mat left_img_, right_img_;   // stereo images
        cv::Mat descriptors_;             // ORB特征描述子
        std::mutex descriptors_mutex_;   // ORB特征描述子数据锁
        
        // 从左图中提取的特征点
        std::vector<std::shared_ptr<Feature>> features_left_;
        // 和左图特征点相匹配的右图特征点
        std::vector<std::shared_ptr<Feature>> features_right_;

        Frame() {}

        Frame(long id, double time_stamp, const Mat &left, const Mat &right);

        // 返回位姿
        SE3 Pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }
        
        // 设置位姿
        void SetPose(const SE3 &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }

        // 设置ORB特征描述子
        void SetDescriptors(const cv::Mat &descriptors) {
            std::unique_lock<std::mutex> lck(descriptors_mutex_);
            descriptors_ = descriptors;
        }

        // 创建新帧，分配id 
        static Ptr CreateFrame();

        // 设置关键帧并分配并键帧id
        void SetKeyFrame();
    };
}

#endif // FRAME_H