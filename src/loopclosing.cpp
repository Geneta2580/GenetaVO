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

            // 检查 map_ 是否有效
            if (map_ == nullptr) {
                continue;
            }

            active_kfs = map_->GetAllKeyFrames();

            // 检查是否有关键帧
            if (active_kfs.empty()) {
                continue;
            }
            
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
            if (active_kfs.count(ret.Id) == 0) { // 安全检查，防止访问不存在的关键帧
                continue;
            }
            if ((ret.Id) != curr_kf_id && ret.Score > min_score_) {
                if(ret.Id != 0) {
                    if((curr_kf_id - ret.Id) > window_size_) { // 时间一致性检测，检测到回环的帧必须相差一定的时间
                        // std::cout << "当前KF的帧ID: " << curr_kf_id1 << " 可能回环的KF的帧ID: " << active_kfs[ret.Id]->id_ <<
                        //  " 当前KF的ID:" << curr_kf_id << " 可能回环的KF的ID: " << ret.Id << std::endl;
                        if(((curr_kf_id - static_id_) > window_size_)) { // 当检测到第一次回环后，只有隔一段时间再检测到回环才算
                            if (RANSAC(curr_kf_id, ret.Id)) { // RANSAC几何校验
                                static_id_ = curr_kf_id;
                                std::unique_lock<std::shared_mutex> loop_id_lock(loop_id_mutex_);
                                loop_id_.insert(std::make_pair(curr_kf_id, ret.Id)); // 插入回环的当前帧和候选帧关键帧ID
                                std::cout << "当前KF的帧ID: " << curr_kf_id1 << " 可能回环的KF的帧ID: " << active_kfs[ret.Id]->id_ <<
                                " 当前KF的ID:" << curr_kf_id << " 可能回环的KF的ID: " << ret.Id << std::endl;
                                std::cout << "回环为真" << std::endl;
                                return true;
                            }
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

        // std::cout << "running" << std::endl;
        cv::BFMatcher matcher(cv::NORM_HAMMING); // 汉明距离
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(desc1, desc2, knn_matches, 2); // 使用KNN匹配

        // Lowe's Ratio Test筛选匹配点，若第一个匹配点距离小于第二个匹配点距离的0.7倍，则认为是好的匹配
        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        if (good_matches.size() < 20) { // 如果好的匹配点太少，直接认为是误匹配
            return false;
        }

        // 转换为Point2f格式
        std::vector<cv::Point2f> pts1, pts2;
        for (auto& m : good_matches) {
            pts1.push_back(kp1[m.queryIdx].pt);
            pts2.push_back(kp2[m.trainIdx].pt);
        }

        // RANSAC计算本质矩阵
        cv::Mat E, mask;
        // 注意：这里的相机内参应该是从配置中读取的真实值
        cv::Mat K = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
        E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

        // 统计内点数量
        int inliers = cv::countNonZero(mask);

        // 判断相似性（内点数量>阈值则认为几何一致）
        const int min_inliers = 15; 
        if (inliers > min_inliers) {
            return true;
        } 
        else {
            return false;
        }
    }
}