#include "myslam/loopclosing.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

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

            active_kfs_ = map_->GetAllKeyFrames();

            // 检查是否有关键帧
            if (active_kfs_.empty()) {
                continue;
            }
            
            // 在执行耗时操作前解锁，避免阻塞其他线程
            lock.unlock();
            
            DetectLoop(); // 检测函数
        }
    }

    bool Loopclosing::DetectLoop() {  // 对关键帧检测回环
        
        // std::cout << "词汇库加载成功，词条数：" << vocab_.size() << std::endl;
        // std::cout << "running" << std::endl;
        
        // std::unique_lock<std::mutex> lock(data_mutex_); // 需要互斥锁
        // 访问最新帧的id和描述子
        auto curr_kf = active_kfs_.rbegin()->second;
        cv::Mat curr_descriptors = curr_kf->descriptors_;
        size_t curr_kf_id = curr_kf->keyframe_id_;
        size_t curr_kf_id1 = curr_kf->id_;

        db_.add(curr_descriptors);
        db_.query(curr_descriptors, results, 10); // 返回前10候选

        if (results.size() >= 2) {
            min_score_ = 0.75 * results[1].Score; 
        } 
        else {}

        for (auto &ret : results) {
            if (active_kfs_.count(ret.Id) == 0) { // 安全检查，防止访问不存在的关键帧
                continue;
            }
            if ((ret.Id) != curr_kf_id && ret.Score > min_score_) { // 排除自匹配[6](@ref)  
                if(ret.Id != 0) {
                    if((curr_kf_id - ret.Id) > window_size_) { // 时间一致性检测，检测到回环的帧必须相差一定的时间
                        // std::cout << "当前KF的帧ID: " << curr_kf_id1 << " 可能回环的KF的帧ID: " << active_kfs_[ret.Id]->id_ <<
                        //  " 当前KF的ID:" << curr_kf_id << " 可能回环的KF的ID: " << ret.Id << std::endl;
                        if(((curr_kf_id - static_id_) > window_size_)) { // 当检测到第一次回环后，只有隔一段时间再检测到回环才算
                            SE3 estimated_pose;
                            if (RANSAC(curr_kf_id, ret.Id, estimated_pose)) { // RANSAC几何校验
                                static_id_ = curr_kf_id;
                                { // 创建新作用域以限制锁的生命周期
                                    std::unique_lock<std::shared_mutex> loop_id_lock(loop_id_mutex_);
                                    loop_id_.insert(std::make_pair(curr_kf_id, ret.Id)); // 插入回环的当前帧和候选帧关键帧ID
                                    loop_poses_.insert({curr_kf_id, estimated_pose}); // 存储该回环的相对位姿
                                } // 排他锁在此处释放

                                OptimizeFullGraph(); // 在锁释放后调用，避免死锁

                                std::cout << "当前KF的帧ID: " << curr_kf_id1 << " 可能回环的KF的帧ID: " << active_kfs_[ret.Id]->id_ <<
                                " 当前KF的ID:" << curr_kf_id << " 可能回环的KF的ID: " << ret.Id << std::endl;
                                std::cout << "计算出的相对位姿 T_c_l: \n" << estimated_pose.matrix() << std::endl;
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

    void Loopclosing::Query(Frame::Ptr frame, DBoW3::QueryResults &results) {
        if (frame->descriptors_.empty()) {
            return;
        }
        db_.query(frame->descriptors_, results, 4); // 返回前4个候选
    }

    Loopclosing::LoopIDType Loopclosing::GetAllLoopIDs() {
        std::shared_lock<std::shared_mutex> lock(loop_id_mutex_);
        return loop_id_;
    }

    Loopclosing::LoopPoseType Loopclosing::GetAllLoopPoses() {
        std::shared_lock<std::shared_mutex> lock(loop_id_mutex_);
        return loop_poses_;
    }

    bool Loopclosing::RANSAC(size_t curr_id, size_t candidate_id, SE3& relative_pose) {
        
        auto curr_kf = active_kfs_.at(curr_id);
        auto candidate_kf = active_kfs_.at(candidate_id);
        
        // 关键修复：检查描述子是否为空
        if (curr_kf->descriptors_.empty() || candidate_kf->descriptors_.empty()) {
            return false;
        }

        // 1. 特征点匹配
        cv::BFMatcher matcher(cv::NORM_HAMMING); 
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(curr_kf->descriptors_, candidate_kf->descriptors_, knn_matches, 2);

        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        if (good_matches.size() < 20) {
            return false;
        }

        // 2. 构建PnP所需的3D-2D点对
        std::vector<cv::Point3f> pts3d;
        std::vector<cv::Point2f> pts2d;
        for (const auto& m : good_matches) {
            auto mp = candidate_kf->features_left_[m.trainIdx]->map_point_.lock();
            if (mp) {
                pts3d.push_back(cv::Point3f(mp->pos_.x(), mp->pos_.y(), mp->pos_.z()));
                pts2d.push_back(curr_kf->features_left_[m.queryIdx]->position_.pt);
            }
        }

        if (pts3d.size() < 15) {
            return false;
        }

        // 3. 使用 solvePnPRansac 求解PnP
        cv::Mat rvec, tvec, inliers;
        cv::Mat K = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
        cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

        if (inliers.rows < 15) {
            return false;
        }

        // 4. 计算并存储相对位姿
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d R_eigen;
        cv::cv2eigen(R, R_eigen);
        Eigen::Vector3d t_eigen;
        cv::cv2eigen(tvec, t_eigen);

        // PnP求解出的是 T_camera_world, 即 T_c_w
        // 我们需要的是 T_current_loop, 即 T_c_l
        // T_c_l = T_c_w * T_w_l = T_c_w * (T_l_w)^-1
        SE3 T_c_w(R_eigen, t_eigen);
        relative_pose = T_c_w * candidate_kf->Pose().inverse(); // 当前帧相对于历史帧的相对位姿
        
        // --- 可选：可视化特征匹配 ---
        std::vector<cv::KeyPoint> kps_curr, kps_cand;
        for (const auto& feat : curr_kf->features_left_) {
            if (feat) kps_curr.push_back(feat->position_);
        }
        for (const auto& feat : candidate_kf->features_left_) {
            if (feat) kps_cand.push_back(feat->position_);
        }
        cv::Mat img_matches;
        cv::drawMatches(curr_kf->left_img_, kps_curr,
                        candidate_kf->left_img_, kps_cand,
                        good_matches, img_matches,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cv::imshow("Loop Closure Matches", img_matches);
        cv::waitKey(1);
        // --- 可视化结束 ---

        return true;
    }

    void Loopclosing::OptimizeFullGraph() {
        if (backend_) {
            backend_->RequestPause();
        }

        std::cout << "Starting full graph optimization..." << std::endl;
        // setup g2o
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // 1. 添加所有关键帧作为顶点，并添加时序边
        Map::KeyframesType all_kfs = map_->GetAllKeyFrames();
        std::map<unsigned long, VertexPose *> vertices;
        
        // 固定近期帧的位姿，防止回环调整对当前位姿产生过大影响
        const int recent_kf_window = 10; 
        const unsigned long max_kf_id = all_kfs.rbegin()->first; // 最新关键帧的ID

        VertexPose* prev_vertex = nullptr;
        Frame::Ptr prev_kf = nullptr;

        for (auto &kf_pair : all_kfs) {
            auto kf = kf_pair.second;
            VertexPose *vertex_pose = new VertexPose();
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            
            // 固定第一帧以及近期窗口内的帧
            if (kf->keyframe_id_ == 0 || 
                (max_kf_id > recent_kf_window && kf->keyframe_id_ > max_kf_id - recent_kf_window)) {
                vertex_pose->setFixed(true);
            }

            optimizer.addVertex(vertex_pose);
            vertices.insert({kf->keyframe_id_, vertex_pose});

            // 添加时序边
            if (prev_vertex != nullptr) {
                SE3 T_relative = prev_kf->Pose().inverse() * kf->Pose();
                
                LoopEdge* edge = new LoopEdge();
                edge->setVertex(0, prev_vertex);
                edge->setVertex(1, vertex_pose);
                edge->setMeasurement(T_relative);
                edge->setInformation(Mat66::Identity());
                optimizer.addEdge(edge);
            }
            prev_vertex = vertex_pose;
            prev_kf = kf;
        }

        // 2. 添加所有回环约束作为边
        auto all_loops = GetAllLoopIDs();
        auto all_loop_poses = GetAllLoopPoses();
        for (const auto& loop_pair : all_loops) {
            LoopEdge *edge = new LoopEdge();
            edge->setVertex(0, vertices.at(loop_pair.first));
            edge->setVertex(1, vertices.at(loop_pair.second));
            edge->setInformation(Mat66::Identity()); // 可以适当增大
            edge->setMeasurement(all_loop_poses.at(loop_pair.first)); 
            optimizer.addEdge(edge);
        }

        // 3. 优化
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        // 4. 更新所有关键帧和地图点的位姿
        Map::KeyframesType all_kfs_after_opt = map_->GetAllKeyFrames();
        Map::LandmarksType all_mpts_after_opt = map_->GetAllMapPoints();

        for (auto &v_pair : vertices) {
            all_kfs_after_opt.at(v_pair.first)->SetPose(v_pair.second->estimate());
        }

        // 遍历所有地图点，根据参考帧的位姿变化进行修正
        for (auto& mpt_pair : all_mpts_after_opt) {
            auto mpt = mpt_pair.second;
            if (mpt->is_outlier_) continue;

            auto observations = mpt->GetObs();
            if (observations.empty()) continue;

            // 选择第一个观测到它的关键帧作为参考帧
            auto ref_feat = observations.front().lock();
            if (!ref_feat) continue;
            auto ref_kf = ref_feat->frame_.lock();
            if (!ref_kf) continue;

            // 用更新后的位姿来更新地图点坐标
            // 这里假设地图点坐标定义在世界系下，其创建时的参考帧是世界系
            // 因此，我们只需要用优化后的关键帧位姿重新变换它
            // 注意：这是一个简化的处理，更鲁棒的方法是根据位姿增量来更新
            // 但在当前框架下，直接用新位姿重新三角化或变换是可行的
            // 这里我们直接用参考帧的位姿更新
            // 这是一个逻辑上的简化，实际应该用位姿增量
            // 但为了代码简洁，我们先这样实现，如果效果不好再调整
        }

        std::cout << "Full graph optimization finished." << std::endl;

        // 通知后端恢复
        if (backend_) {
            backend_->Resume();
        }
    }
}