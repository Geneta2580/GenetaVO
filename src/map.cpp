#include "myslam/map.h"

namespace myslam {
  
    void Map::InsertKeyFrame(Frame::Ptr frame) {
        std::lock_guard<std::mutex> lock(keyframes_mutex_); // 加锁
        current_frame_ = frame;
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
            keyframes_.insert(make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        } else {
            keyframes_[frame->keyframe_id_] = frame;  // 对帧图像进行更新
            active_keyframes_[frame->keyframe_id_] = frame;
        }
    
        if (active_keyframes_.size() > num_active_frames_) { // 大于指定的活跃关键帧数量
            RemoveOldKeyframe();
        }
    }

    void Map::InsertMapPoint(MapPoint::Ptr map_point) {
        std::lock_guard<std::mutex> lock(landmarks_mutex_);  // 加锁

        if (landmarks_.find(map_point->id_) == landmarks_.end()) {
            landmarks_.insert(make_pair(map_point->id_, map_point));
            active_landmarks_.insert(make_pair(map_point->id_, map_point));
            
        } else {       
            landmarks_[map_point->id_] = map_point; // 对地图路标点进行更新，相同id的mappoint被观测了多次？
            active_landmarks_[map_point->id_] = map_point;
        }
    }
    
    void Map::RemoveOldKeyframe() {
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
                mp->RemoveObservation(feat); // 删掉指定帧的左特征点观测
            }
        }
        for (auto feat : frame_to_remove->features_right_) {
            if (feat == nullptr) continue; // 右图特征点在光流追上一帧左图的时候为空
            auto mp = feat->map_point_.lock();
            if (mp) {
                mp->RemoveObservation(feat); // 删掉指定帧的右特征点观测
            }
        }
    
        CleanMap();
    }

    void Map::CleanMap() {
        int cnt_landmark_removed = 0;
        for (auto iter = active_landmarks_.begin();
             iter != active_landmarks_.end();) {
            if (iter->second->observed_times_ == 0) { // 清除观测次数为零的点
                iter = active_landmarks_.erase(iter);
                cnt_landmark_removed++;
            } else {
                ++iter;
            }
        }
        // std::cout << "Removed " << cnt_landmark_removed << " active landmarks" << std::endl;
    }

}