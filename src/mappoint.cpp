#include "myslam/mappoint.h"

namespace myslam {

    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

    MapPoint::Ptr MapPoint::CreateNewMappoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++; // 创建成功+1
        return new_mappoint;
    }

    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feat) { // 去掉一个观测点
        std::unique_lock<std::mutex> lck(data_mutex_); // 添加了锁
        for (auto iter = observations_.begin(); iter != observations_.end();
            iter++) {
            if (iter->lock() == feat) {
                observations_.erase(iter);
                feat->map_point_.reset();
                observed_times_--;
                break;
            }
        }
    }

}