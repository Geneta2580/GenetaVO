#pragma once

#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "myslam/common.h"
#include "myslam/frame.h"
#include "myslam/feature.h"
#include "myslam/config.h"
#include "myslam/algorithm.h"
#include "myslam/viewer.h"
#include "myslam/camera.h"
#include "myslam/g2o_param.h"
#include "myslam/backend.h"
#include "myslam/loopclosing.h"

namespace myslam {

    // 前端工作状态
    enum class FrontendStatus {INIT, TRACK, KEYFRAME}; 

    // 前端，估计当前帧的Pose，并在适当时刻向地图中加入地图点
    class Frontend { 

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Frontend> Ptr;
     
            Frontend();

            // 获取前端状态
            FrontendStatus GetStatus() const { return status_; }

            // 添加一个帧并进行计算
            bool Calculate(Frame::Ptr frame);

            // Set函数，用来给private变量赋值（外部操作frontend成员的接口）
            void SetMap(Map::Ptr map) { map_ = map; }

            void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

            void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

            void SetCameras(Camera::Ptr left, Camera::Ptr right) {
                camera_left_ = left;
                camera_right_ = right;
            }

            // 停止后端、可视化界面工作
            void Stop();

        private:

            // 初始化前端状态
            FrontendStatus status_ = FrontendStatus::INIT; 

            // 进行数据初始化
            bool Init(); 

            // 进行左目图像GFTT特征点提取，返回提取的特征点数量
            int DetectLeftFeatures(); 

            // 进行光流追踪，返回在右目图像中追踪到的特征点数量
            int FindRightFeatures(); 

            // 进行光流追踪，通过上一帧特征点来检测这一帧的特征点
            int LKDetectLastFeatures();

            // 三角化，通过左右目图像匹配点计算空间点，返回三角化成功的匹配点值
            int Triangulation();

            // 进行PnP问题的求解，得到这一帧图像和上一帧图像的相对位姿
            int BACurrentPose();

            // 初始化第一帧的地图
            bool MapInit();

            // 进行光流追踪、位姿估计
            int Track();         

            // 重新进行特征点检测以及三角化
            void ReTrack();

            // 插入关键帧
            void InsertKeyFrame();

            // 数据变量初始化，注意这里不能为空指针，因为在前端程序中会有访问初始值的成员变量，不能访问空指针的成员变量
            Frame::Ptr current_frame_ = nullptr;  // 当前帧 std::make_shared<Frame>()这个可能导致线程的问题
            Frame::Ptr last_frame_ = nullptr;     // 上一帧
            Camera::Ptr camera_left_ = nullptr;   // 左侧相机
            Camera::Ptr camera_right_ = nullptr;  // 右侧相机
            Map::Ptr map_ = nullptr; // 全局地图
            BackendPtr backend_ = nullptr; // 后端
            Viewer::Ptr viewer_ = nullptr; // 可视化界面
            LoopclosingPtr loop_ = nullptr; // 回环

            // 上一帧到当前帧的相对运动，用于估计当前帧pose初值，默认初始化为单位阵
            SE3 relative_motion_;

            // 隔一帧插入关键帧的标志
            bool insert_key_flag = true;

            // 一些参数
            int num_features_ = Config::Get<int>("num_features");
            int num_features_tracking_ = myslam::Config::Get<double>("num_features_tracking"); 
            int num_track_good_ = 0;

            int num_features_init_ = 100;
            int num_features_needed_for_keyframe_ = 80;
    };
}
#endif