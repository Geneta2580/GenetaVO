#include "myslam/loopclosing.h"

namespace myslam{

    Loopclosing::Loopclosing() : db_(vocab_, false, 0) {
        vocab_.load("/home/geneta/Project/geneta_slam/Thirdparty/DBow3/orbvoc.dbow3");
        db_.setVocabulary(vocab_);  // 假设Database提供重新加载接口
        loop_running_.store(true);
        loop_thread_ = std::thread(std::bind(&Loopclosing::Loop, this)); // 启动回环线程
    }
    
    void Loopclosing::Wake() {
        std::unique_lock<std::mutex> lock(data_mutex_); // 需要互斥锁
        loop_update_.notify_one(); // 唤醒线程
    }

    void Loopclosing::Stop() {
        loop_running_.store(false);
        loop_update_.notify_one();
        loop_thread_.join();
    }

    void Loopclosing::Loop() { // 回环主线程
        while (loop_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_); // 等待线程被唤醒
            loop_update_.wait(lock);

            active_kfs = map_->GetActiveKeyFrames();
            // Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            DetectLoop(); // 检测函数
        }
    }

    bool Loopclosing::DetectLoop() {  // 对关键帧检测回环
        
        // std::cout << "词汇库加载成功，词条数：" << vocab_.size() << std::endl;
        // std::cout << "running" << std::endl;
        
        // std::unique_lock<std::mutex> lock(data_mutex_); // 需要互斥锁
        // 访问最新帧的id和描述子
        auto curr_kf = active_kfs.rbegin()->second;
        cv::Mat curr_descriptors = curr_kf->descriptors_;
        size_t curr_kf_id = curr_kf->keyframe_id_;
        size_t curr_kf_id1 = curr_kf->id_;

        db_.add(curr_descriptors);
        db_.query(curr_descriptors, results, 10); // 返回前10候选

        if (results.size() >= 2) {
            min_score_ = 0.75 * results[1].Score; // 排除自匹配[6](@ref)  
        } 
        else {}

        for (auto &ret : results) {
            if ((ret.Id) != curr_kf_id && ret.Score > min_score_) {
                if(ret.Id != 0) {
                    if((curr_kf_id - ret.Id) > window_size_) { // 时间一致性检测，检测到回环的帧必须相差一定的时间
                        // std::cout << "当前KF的帧ID: " << curr_kf_id1 << " 可能回环的KF的帧ID: " << active_kfs[ret.Id]->id_ <<
                        //  " 当前KF的ID:" << curr_kf_id << " 可能回环的KF的ID: " << ret.Id << std::endl;
                        if(((curr_kf_id - static_id_) > window_size_) && RANSAC(curr_kf_id, ret.Id)) { // 当检测到第一次回环后，只有隔一段时间再检测到回环才算
                            static_id_ = curr_kf_id;
                            std::unique_lock<std::shared_mutex> loop_id_lock(loop_id_mutex_);
                            loop_id_.insert(std::make_pair(curr_kf_id, ret.Id)); // 插入回环的当前帧和候选帧关键帧ID
                            // std::cout << "回环为真" << std::endl;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    bool Loopclosing::RANSAC(size_t curr_id, size_t candidate_id) {
        
        cv::Mat desc1 = active_kfs[curr_id]->descriptors_;
        cv::Mat desc2 = active_kfs[candidate_id]->descriptors_;
        std::vector<cv::KeyPoint> kp1;
        for(auto &feature: active_kfs[curr_id]->features_left_) {
            kp1.push_back(feature->position_);
        } 
        std::vector<cv::KeyPoint> kp2;
        for(auto &feature: active_kfs[candidate_id]->features_left_) {
            kp2.push_back(feature->position_);
        } 

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(desc1, desc2, matches);

        // 转换为Point2f格式
        std::vector<cv::Point2f> pts1, pts2;
        for (auto& m : matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }

        // RANSAC计算单应性矩阵
        cv::Mat mask;
        const double ransac_thresh = 3.0;  // 重投影误差阈值（像素）
        cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, ransac_thresh, mask);

        // 统计内点数量、比例
        int inliers = cv::countNonZero(mask);
        double inlier_ratio = static_cast<double>(inliers) / matches.size();

        // 判断相似性（内点比例>阈值则认为相似）
        const double similarity_threshold = 0.3; // 室外场景这个值可以适当降低
        if (inlier_ratio > similarity_threshold) {return true;} 
        else {return false;}
    }
}