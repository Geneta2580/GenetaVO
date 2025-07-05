#pragma once

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common.h"
#include "myslam/feature.h"

namespace myslam {

    struct Frame;
    struct Feature;

    struct MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long id_ = 0;  // ID
        bool is_outlier_ = false; // 是否越界
        Vec3 pos_ = Vec3::Zero();  // 点的世界3D坐标
        std::mutex data_mutex_; // 数据锁

        int observed_times_ = 0;  // 在不同帧间同一点被观测到的次数
        std::list<std::weak_ptr<Feature>> observations_; // 观测值是一个Feature类的列表，相邻关键帧可能存在共视关系

        MapPoint() {}

        MapPoint(long id, Vec3 position);

        // 获取mappoint值
        Vec3 Pos() {               
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        // 设置mappoint值
        void SetPos(const Vec3 &pos) {         
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        };

        // 同一点被观测到的计数，已实现，并将匹配的地图点和特征点进行存储
        void AddObservation(std::shared_ptr<Feature> feature) {
            observations_.push_back(feature);
            observed_times_++; 
        }

        // 移除一个观测点，未实现
        void RemoveObservation(std::shared_ptr<Feature> feat); 

        // 获取观测点，已实现
        std::list<std::weak_ptr<Feature>> GetObs() { 
            return observations_;
        }

        // 创建一个新的地图点，未实现
        static MapPoint::Ptr CreateNewMappoint();
    };
}

#endif
