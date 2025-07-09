#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

#include "myslam/common.h"
#include "myslam/map.h"
#include "myslam/frame.h"
#include "myslam/camera.h"
#include "DBoW3.h"
#include "myslam/backend.h"
#include "myslam/g2o_param.h"
#include "myslam/algorithm.h"
// #include <DBoW3/Vocabulary.h>

namespace myslam{
    class Backend;  // 前置声明
    using BackendPtr = std::shared_ptr<Backend>; // 独立类型声明[5](@ref)

    class Loopclosing {
        friend class Backend;  // 允许Backend访问私有成员

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Loopclosing>; // C++11方式
            
            // 将LoopIDType重构为可以存储每个回环的相对位姿
            typedef std::map<size_t, SE3> LoopPoseType;
            typedef std::map<size_t, size_t> LoopIDType;

            // 构造函数中启动回环线程并挂起
            Loopclosing();

            // 关闭回环线程
            void Stop();

            void InsertNewKeyFrame(Frame::Ptr kf);

            // 设置地图，用于传递map参数
            void SetMap(std::shared_ptr<Map> map) { map_ = map; }

            // 对关键帧和路标点进行回环检测
            bool DetectLoop();

            // 提供给前端进行重定位查询
            void Query(Frame::Ptr frame, DBoW3::QueryResults &results);

            // 获取所有已确认的回环ID对
            LoopIDType GetAllLoopIDs();

            // 获取所有回环的相对位姿
            LoopPoseType GetAllLoopPoses();

            // 候选的关键帧ID
            LoopIDType loop_id_;

            // 关联后端
            void SetBackend(BackendPtr backend) { backend_ = backend; }

        private:
            // 对可能的回环做空间一致RANSAC检测, 成功时返回相对位姿
            bool RANSAC(size_t curr_id, size_t candidate_id, SE3& relative_pose);

            // 进行回环全局优化
            void OptimizeFullGraph();

            // 存储每个回环的相对位姿，键为当前帧ID
            LoopPoseType loop_poses_;

            DBoW3::Vocabulary vocab_; // 持久化词汇库
            DBoW3::Database db_;       // 数据库实例
            DBoW3::QueryResults results; 

            // 传入的关键帧
            Map::KeyframesType all_kfs_;

            // DBoW3内部ID到我们系统关键帧ID的映射表
            std::map<DBoW3::EntryId, size_t> dbow_id_to_kf_id_;

            // 回环主线线程
            void Loop();

            // 全局地图，需要多次传输，重要
            std::shared_ptr<Map> map_;

            // 回环检测线程
            std::thread loop_thread_; 

            // 数据锁
            std::mutex data_mutex_; 

            std::shared_mutex loop_id_mutex_;  // 读写锁，同时保护loop_id_和loop_poses_

            // 满足条件时唤醒线程
            std::condition_variable loop_update_; 

            std::atomic<bool> loop_running_;

            // [任务队列，用于接收来自后端的新关键帧
            std::list<Frame::Ptr> new_keyframes_list_;

            // 建立LoopClosing模块自己的关键帧数据库
            std::map<unsigned long, Frame::Ptr> key_frame_database_;

            // 当前关键帧
            Frame::Ptr current_kf_ = nullptr;

            // 重复的回环计数
            int probably_cnt_ = 0;

            // 用于间隔关键帧
            size_t static_id_ = 0;

            // 关键帧间隔窗口
            int window_size_ = 300;

            // 相似分数阈值
            double min_score_ = 0.02;

            // 最小开始检测的数据库大小
            size_t min_db_size_ = 50; // 新增，避免在数据库过小的情况下进行回环检测

            BackendPtr backend_; // 新增后端实例引用

    };
}

#endif