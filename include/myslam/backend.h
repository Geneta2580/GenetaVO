#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "loopclosing.h"
#include <g2o/types/slam3d/edge_se3.h> // g2o边
#include <mutex>
#include <condition_variable>

namespace myslam {
    class Map;
    class Loopclosing;  // 前置声明
    using LoopclosingPtr = std::shared_ptr<Loopclosing>;

    class Backend {
        friend class Loopclosing; // 添加友元声明
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        using Ptr = std::shared_ptr<Backend>;

        // 构造函数中启动优化线程并挂起
        Backend();

        // 设置左右目的相机，用于获得内外参，给空指针赋值
        void SetCameras(Camera::Ptr left, Camera::Ptr right) {
            cam_left_ = left;
            cam_right_ = right;
        }

        // 设置地图
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        // 启动后端线程
        void Start();

        // 新增：前端用于插入新关键帧的接口
        void InsertNewKeyFrame(Frame::Ptr keyframe);

        // 关闭后端线程
        void Stop();

        // 请求暂停后端
        void RequestPause();

        // 恢复后端
        void Resume();

        // 请求一次全局位姿图优化
        void RequestFullGraphOptimization();

        // 关联回环
        void SetLoopclosing(LoopclosingPtr loop) { loop_ = loop; }

    private:
        // 后端线程
        void BackendLoop();

        // 新增：用于处理队列中新关键帧的函数
        void ProcessNewKeyFrames();

        // 全局位姿图优化
        void OptimizeFullGraph();

        // 对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;
        std::mutex pause_mutex_;  // 暂停线程锁

        // std::condition_variable map_update_; // 移除条件变量
        std::condition_variable resume_cv_; // 用于暂停和恢复
        std::atomic<bool> backend_running_;
        std::atomic<bool> pause_requested_{false};

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

        LoopclosingPtr loop_;

        // 新增：用于前后端通信的线程安全队列
        std::list<Frame::Ptr> new_keyframes_list_;
        std::mutex new_keyframes_mutex_;
        std::condition_variable new_kf_cv_;
    };

}

#endif