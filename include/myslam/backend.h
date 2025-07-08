#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common.h"
#include "myslam/frame.h"
#include "myslam/map.h"
#include "myslam/camera.h"
#include "loopclosing.h"
#include <g2o/types/slam3d/edge_se3.h> // g2o边

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

        // 触发地图更新，启动优化
        void Wake();

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

        // 全局位姿图优化
        void OptimizeFullGraph();

        // 对给定关键帧和路标点进行优化
        void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;
        std::mutex pause_mutex_;  // 暂停线程锁

        std::condition_variable map_update_; // 满足条件时唤醒线程
        std::condition_variable resume_cv_; // 用于暂停和恢复
        std::atomic<bool> backend_running_;
        std::atomic<bool> pause_requested_{false};

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;

        LoopclosingPtr loop_; // 新增后端实例引用

        int old_vertex_id = 0; // 需要边缘化的旧顶点

        // 插入的新帧地图点
        Frame::Ptr new_key_frame_;
        MapPoint::Ptr new_map_point_;
    };

}

#endif