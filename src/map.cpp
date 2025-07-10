#include "myslam/map.h"

namespace myslam {
  
    void Map::InsertKeyFrame(Frame::Ptr frame) {
        current_frame_ = frame;
        
        std::unique_lock<std::mutex> lck(data_mutex_);
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;  // 对帧图像进行更新
            active_keyframes_[frame->keyframe_id_] = frame;
        }
    
        // 添加新关键帧的活跃观测
        for (auto& feat : frame->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->AddActiveObservation(feat);
                InsertActiveMapPoint(mp);
            }
        }

        if (active_keyframes_.size() > num_active_frames_) { // 大于指定的活跃关键帧数量
            RemoveOldKeyframe();
            RemoveOldActiveMapPoints();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        if (landmarks_.find(map_point->id_) == landmarks_.end()) {
            landmarks_.insert(make_pair(map_point->id_, map_point));
        } else {       
            landmarks_[map_point->id_] = map_point;
        }
    }

    void Map::InsertActiveMapPoint(MapPoint::Ptr map_point) {
        // 外部已加锁，这里不再加锁
        if (active_landmarks_.find(map_point->id_) == active_landmarks_.end()) {
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
        } else {
            active_landmarks_[map_point->id_] = map_point;
        }
    }
    
    void Map::RemoveOldKeyframe() {
        // 外部已加锁，这里不再加锁
        if (current_frame_ == nullptr) return;
        // 寻找与当前帧最近与最远的两个关键帧
        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;
        auto Twc = current_frame_->Pose().inverse();
        for (auto& kf : active_keyframes_) {
            if (kf.second == current_frame_) continue;
            auto dis = (kf.second->Pose() * Twc).log().norm(); // 将两帧之间的平移变量作为距离指标
            if (dis > max_dis) {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if (dis < min_dis) {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }
    
        const double min_dis_th = 0.2;  // 最近阈值
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th) {
            frame_to_remove = active_keyframes_.at(min_kf_id); // 如果存在很近的帧，指定最近的
        } else {
            frame_to_remove = active_keyframes_.at(max_kf_id); // 其它情况则指定最远的
        }
    
        // std::cout << "remove keyframe " << frame_to_remove->keyframe_id_ << std::endl;

        // remove keyframe and landmark observation
        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        for (auto feat : frame_to_remove->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveActiveObservation(feat); // 移除活跃观测
            }
        }
        for (auto feat : frame_to_remove->features_right_) {
            if (feat == nullptr) continue; // 右图特征点在光流追上一帧左图的时候为空
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveActiveObservation(feat); // 移除活跃观测
            }
        }
    }

    void Map::RemoveOldActiveMapPoints() {
        // 外部已加锁，这里不再加锁
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin();
             iter != active_landmarks_.end();) {
            if (iter->second->active_observed_times_ == 0) { // 清除活跃观测次数为零的点
                iter = active_landmarks_.erase(iter);
                cnt_landmark_removed++;
            } else {
                ++iter;
            }
        }
        // std::cout << "Removed " << cnt_landmark_removed << " active landmarks" << std::endl;
    }

    void Map::AddOutlierMapPoint(unsigned long mpId) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        outlier_mappoints_.insert(mpId);
    }

    void Map::RemoveAllOutlierMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto id : outlier_mappoints_) {
            landmarks_.erase(id);
            active_landmarks_.erase(id);
        }
        outlier_mappoints_.clear();
    }

    Map::LandmarksType Map::GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    Map::KeyframesType Map::GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    Map::LandmarksType Map::GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    Map::KeyframesType Map::GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // --- [新增] GetKeyFrame 方法的实现 ---
    Frame::Ptr Map::GetKeyFrame(unsigned long id) {
        // 注意：这个函数本身不加锁，调用者需要在外部保证锁的存在
        // 因为它通常在已经持有锁的上下文中被调用
        auto it = keyframes_.find(id);
        if (it != keyframes_.end()) {
            return it->second;
        }
        return nullptr;
    }
}