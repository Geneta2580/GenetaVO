#pragma once

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"
#include <mutex>
#include <unordered_set>

namespace myslam {

    class Map {

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Map> Ptr;
            typedef std::map<unsigned long, MapPoint::Ptr> LandmarksType; // 路标类型为一个id加上一个地图点
            typedef std::map<unsigned long, Frame::Ptr> KeyframesType; // 帧类型为一个id加上一个帧类
        
            Map() {}
        
            /// 公开的、用于模块间协调的粗粒度锁
            std::mutex map_update_mutex_;

            /// 增加一个关键帧
            void InsertKeyFrame(Frame::Ptr frame);

            /// 增加一个地图顶点
            void InsertMapPoint(MapPoint::Ptr map_point);

            /// 获取所有地图点
            LandmarksType GetAllMapPoints();

            /// 获取所有关键帧
            KeyframesType GetAllKeyFrames();
        
            /// 获取激活地图点
            LandmarksType GetActiveMapPoints();
        
            /// 获取激活关键帧
            KeyframesType GetActiveKeyFrames();

            // 添加地图外点列表
            void AddOutlierMapPoint(unsigned long mpId);

            // 移除地图外点
            void RemoveAllOutlierMapPoints();

            /// 清理map中活跃观测数量为零的点
            void RemoveOldActiveMapPoints();

            // --- [新增] 添加一个线程安全的方法来获取单个关键帧 ---
            Frame::Ptr GetKeyFrame(unsigned long id);
			
        private:
            void InsertActiveMapPoint(MapPoint::Ptr map_point);
            // 将旧的关键帧置为不活跃状态
            void RemoveOldKeyframe();
        
            /// 私有的、用于保护内部数据容器的细粒度锁
            std::mutex data_mutex_;
            
            LandmarksType landmarks_;         // all landmarks
            LandmarksType active_landmarks_; 
            KeyframesType keyframes_;         // all keyframes
            KeyframesType active_keyframes_;
        
            std::unordered_set<unsigned long> outlier_mappoints_; // 越界的地图点id列表
            Frame::Ptr current_frame_ = nullptr;
        
            // 激活帧（用于BA优化）的数量
			size_t num_active_frames_ = 12;  
    };

}

#endif