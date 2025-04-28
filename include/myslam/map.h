#pragma once

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "myslam/common.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"
#include <pangolin/pangolin.h>

namespace myslam {

    class Map {

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<Map> Ptr;
            typedef std::map<unsigned long, MapPoint::Ptr> LandmarksType; // 路标类型为一个id加上一个地图点
            typedef std::map<unsigned long, Frame::Ptr> KeyframesType; // 帧类型为一个id加上一个帧类
        
            Map() {}
        
            /// 增加一个关键帧
            void InsertKeyFrame(Frame::Ptr frame);

            /// 增加一个地图顶点
            void InsertMapPoint(MapPoint::Ptr map_point);
        
            /// 获取所有地图点
            LandmarksType GetAllMapPoints() {
                std::unique_lock<std::mutex> lck(data_mutex_);
                return landmarks_;
            }

            /// 获取所有关键帧
            KeyframesType GetAllKeyFrames() {
                std::unique_lock<std::mutex> lck(data_mutex_);
                return keyframes_;
            }
        
            /// 获取激活地图点
            LandmarksType GetActiveMapPoints() {
                std::unique_lock<std::mutex> lck(data_mutex_);
                return active_landmarks_;
            }
        
            /// 获取激活关键帧
            KeyframesType GetActiveKeyFrames() {
                std::unique_lock<std::mutex> lck(data_mutex_);
                return active_keyframes_;
            }
        
            /// 清理map中观测数量为零的点
            void CleanMap();
			
        private:
            // 将旧的关键帧置为不活跃状态
            void RemoveOldKeyframe();
        
            std::mutex landmarks_mutex_;  // 独立锁
            std::mutex keyframes_mutex_;  // 独立锁
            
            std::mutex data_mutex_;
            LandmarksType landmarks_;         // all landmarks
            LandmarksType active_landmarks_; 
            KeyframesType keyframes_;         // all keyframes
            KeyframesType active_keyframes_;
        
            Frame::Ptr current_frame_ = nullptr;
        
            // 激活帧（用于BA优化）的数量
			 size_t num_active_frames_ = 7;  
    };

}

#endif