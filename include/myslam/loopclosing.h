#ifndef MYSLAM_LOOPCLOSING_H
#define MYSLAM_LOOPCLOSING_H

#include "myslam/common.h"
#include "myslam/map.h"
#include "myslam/frame.h"
#include "DBoW3.h"
#include "myslam/backend.h"
// #include <DBoW3/Vocabulary.h>

namespace myslam{
    class Backend;  // 前置声明
    using BackendPtr = std::shared_ptr<Backend>; // 独立类型声明[5](@ref)

    class Loopclosing {
        friend class Backend;  // 允许Backend访问私有成员

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            using Ptr = std::shared_ptr<Loopclosing>; // C++11方式
            typedef std::map<size_t, size_t> LoopIDType;

            // 构造函数中启动回环线程并挂起
            Loopclosing();

            // 触发地图更新，启动回环检测
            void Wake();

            // 关闭回环线程
            void Stop();

            // 设置地图，用于传递map参数
            void SetMap(std::shared_ptr<Map> map) { map_ = map; }

            // 对关键帧和路标点进行回环检测
            bool DetectLoop();

            // 对可能的回环做空间一致RANSAC检测
            bool RANSAC(size_t curr_id_, size_t candidate_id);

            // 候选的关键帧ID
            LoopIDType loop_id_;

            // 关联后端
            void SetBackend(BackendPtr backend) { backend_ = backend; }

        private:

            DBoW3::Vocabulary vocab_; // 持久化词汇库
            DBoW3::Database db_;       // 数据库实例
            DBoW3::QueryResults results; 

            // 传入的关键帧
            Map::KeyframesType active_kfs;

            // 回环主线线程
            void Loop();

            // 
            std::shared_ptr<Map> map_;

            // 回环检测线程
            std::thread loop_thread_; 

            // 数据锁
            std::mutex data_mutex_; 

            std::shared_mutex loop_id_mutex_;  // 读写锁

            // 满足条件时唤醒线程
            std::condition_variable loop_update_; 

            std::atomic<bool> loop_running_;

            // 重复的回环计数
            int probably_cnt_ = 0;

            // 用于间隔关键帧
            size_t static_id_ = 0;

            // 关键帧间隔窗口
            int window_size_ = 20;

            // 相似分数阈值
            double min_score_ = 0;

            BackendPtr backend_; // 新增后端实例引用

    };
}

#endif