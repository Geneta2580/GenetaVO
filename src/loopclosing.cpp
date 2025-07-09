#include "myslam/loopclosing.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

namespace myslam{

    Loopclosing::Loopclosing() : db_(vocab_, false, 0) {
        vocab_.load("/home/geneta/Project/geneta_slam/Thirdparty/DBow3/orbvoc.dbow3");
        db_.setVocabulary(vocab_);
        loop_running_.store(true);
        loop_thread_ = std::thread(std::bind(&Loopclosing::Loop, this));
    }
    
    void Loopclosing::InsertNewKeyFrame(Frame::Ptr kf) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        new_keyframes_list_.push_back(kf); // 所有关键帧存储在此
        loop_update_.notify_one();
    }

    void Loopclosing::Stop() {
        loop_running_.store(false);
        loop_update_.notify_one();
        if (loop_thread_.joinable()) {
            loop_thread_.join();
        }
    }

    void Loopclosing::Loop() {
        while (loop_running_.load()) {
            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                loop_update_.wait(lock, [this] {
                    return !new_keyframes_list_.empty() || !loop_running_.load();
                });

                if (!loop_running_.load()) break;

                current_kf_ = new_keyframes_list_.front();
                new_keyframes_list_.pop_front();
            }

            if (current_kf_) {
                key_frame_database_[current_kf_->id_] = current_kf_;
                if (key_frame_database_.size() > min_db_size_) {
                    DetectLoop();
                }     
            }
        }
    }

    bool Loopclosing::DetectLoop() {
        cv::Mat curr_descriptors = current_kf_->descriptors_;
        size_t curr_kf_id = current_kf_->id_;

        // 调用无ID的add函数，获取DBoW3返回的内部ID
        DBoW3::EntryId dbow_id = db_.add(curr_descriptors);
        
        // 在映射表中保存对应关系
        dbow_id_to_kf_id_[dbow_id] = curr_kf_id;

        DBoW3::QueryResults results;
        db_.query(curr_descriptors, results, 10); 

        for (auto &ret : results) {
            // 通过映射表将DBoW3的ID转换为我们的keyframe_id
            if (dbow_id_to_kf_id_.count(ret.Id) == 0) {
                continue; // 如果映射不存在，跳过
            }
            size_t candidate_kf_id = dbow_id_to_kf_id_.at(ret.Id);

            // 安全检查：确保候选帧在我们的数据库中
            if (!key_frame_database_.count(candidate_kf_id)) {
                std::cout << "No candidate_kf in base!" << std::endl;
                continue;
            }

            if (candidate_kf_id != curr_kf_id && ret.Score > min_score_) {
                if(candidate_kf_id != 0) {
                    if((curr_kf_id - candidate_kf_id) > window_size_) {
                        std::cout << "潜在回环: 当前KF ID: " << curr_kf_id 
                                  << ", 候选KF ID: " << candidate_kf_id 
                                  << ", 相似度分数: " << ret.Score << std::endl;
                    
                        SE3 estimated_pose;
                        if (RANSAC(curr_kf_id, candidate_kf_id, estimated_pose)) {
                            static_id_ = curr_kf_id;
                            {
                                std::unique_lock<std::shared_mutex> loop_id_lock(loop_id_mutex_);
                                loop_id_.insert(std::make_pair(curr_kf_id, candidate_kf_id));
                                loop_poses_.insert({curr_kf_id, estimated_pose});
                            }

                            OptimizeFullGraph();

                            std::cout << "回环确认: 当前KF ID: " << curr_kf_id << ", 候选KF ID: " << candidate_kf_id << std::endl;
                            std::cout << "计算出的相对位姿 T_c_l: \n" << estimated_pose.matrix() << std::endl;
                            std::cout << "回环为真" << std::endl;
                            return true;
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
        
        auto curr_kf = key_frame_database_.at(curr_id);
        auto candidate_kf = key_frame_database_.at(candidate_id);
        
        // 1. 特征匹配
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(curr_kf->descriptors_, candidate_kf->descriptors_, knn_matches, 2);

        // 2. 使用 Lowe's ratio test 筛选初始匹配
        const float ratio_thresh = 0.85f;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++) {
            if (knn_matches[i].size() > 1 && knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
                good_matches.push_back(knn_matches[i][0]);
            }
        }

        // --- [新增] 根据您指定的ID范围进行条件可视化 ---
        const size_t debug_start_id = 1571;
        const size_t debug_end_id = 1648;

        // if (curr_id >= debug_start_id && curr_id <= debug_end_id)
        // {
            std::cout << "--- DEBUG VISUALIZATION TRIGGERED ---" << std::endl;
            std::cout << "Current KF ID: " << curr_id << ", Candidate KF ID: " << candidate_id << std::endl;

            if (!good_matches.empty()) {
                std::vector<cv::KeyPoint> kps_curr, kps_cand;
                for(const auto& feat : curr_kf->features_left_) kps_curr.push_back(feat->position_);
                for(const auto& feat : candidate_kf->features_left_) kps_cand.push_back(feat->position_);
                
                cv::Mat img_good_matches;
                cv::drawMatches(curr_kf->left_img_, kps_curr, candidate_kf->left_img_, kps_cand, good_matches, img_good_matches);

                // 在图像上添加调试信息
                std::string text_curr = "Current KF ID: " + std::to_string(curr_id);
                std::string text_cand = "Candidate KF ID: " + std::to_string(candidate_id);
                std::string text_matches = "Good Matches: " + std::to_string(good_matches.size());
                
                int font_face = cv::FONT_HERSHEY_SIMPLEX;
                double font_scale = 0.8;
                int thickness = 2;
                cv::Scalar color(0, 255, 0);

                cv::putText(img_good_matches, text_curr, cv::Point(10, 30), font_face, font_scale, color, thickness);
                cv::putText(img_good_matches, text_cand, cv::Point(curr_kf->left_img_.cols + 10, 30), font_face, font_scale, color, thickness);
                cv::putText(img_good_matches, text_matches, cv::Point(10, 60), font_face, font_scale, color, thickness);

                cv::imshow("Debug Loop Matches", img_good_matches);
                cv::waitKey(1); // 使用waitKey(0)来阻塞线程，等待用户按键
            } else {
                std::cout << "No good matches to display." << std::endl;
            }
        // }
        // --- 可视化代码结束 ---


        if (good_matches.size() < 10) {
            std::cout << "Ratio test后匹配点不足: " << good_matches.size() << std::endl;
            return false;
        }

        // 3. 构建PnP所需的3D-2D点对
        std::vector<cv::Point3f> pts3d_loop;
        std::vector<cv::Point2f> pts2d_curr;
        for (const auto& m : good_matches) {
            auto mp = candidate_kf->features_left_[m.trainIdx]->map_point_.lock();
            if (mp) {
                pts3d_loop.push_back(cv::Point3f(mp->pos_.x(), mp->pos_.y(), mp->pos_.z()));
                pts2d_curr.push_back(curr_kf->features_left_[m.queryIdx]->position_.pt);
            }
        }

        if (pts3d_loop.size() < 15) {
            std::cout << "有效的3D-2D点对不足: " << pts3d_loop.size() << std::endl;
            return false;
        }

        // 4. 使用 solvePnPRansac 求解初始的相对位姿 T_c_l
        cv::Mat rvec, tvec, inliers_cv;
        // 注意：K应该是当前帧的相机内参

        cv::Mat K = (cv::Mat_<double>(3, 3) << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1);
        // PnP求解的是当前帧(c)相对于候选帧(l)的位姿 T_c_l
        cv::solvePnPRansac(pts3d_loop, pts2d_curr, K, cv::Mat(), rvec, tvec, false, 100, 5.991, 0.99, inliers_cv);

        if (inliers_cv.rows < 15) {
            std::cout << "RANSAC求解的内点不足: " << inliers_cv.rows << std::endl;
            return false;
        }

        // 5. 使用g2o对相对位姿 T_c_l 进行精化
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 顶点：待优化的相对位姿 T_c_l
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv);
        Eigen::Matrix3d R_eigen;
        cv::cv2eigen(R_cv, R_eigen);
        Eigen::Vector3d t_eigen;
        cv::cv2eigen(tvec, t_eigen);
        SE3 T_c_l_initial(R_eigen, t_eigen);
        vertex_pose->setEstimate(T_c_l_initial);
        optimizer.addVertex(vertex_pose);

        Mat33 K_eigen;
        cv::cv2eigen(K, K_eigen);

        // 边：投影误差。误差函数 reproject(T_c_l * p_local)
        std::vector<EdgeProjectionPoseOnly*> edges;
        for (int i = 0; i < inliers_cv.rows; ++i) {
            int idx = inliers_cv.at<int>(i, 0);
            Vec3 pt3d_eigen(pts3d_loop[idx].x, pts3d_loop[idx].y, pts3d_loop[idx].z);
            EdgeProjectionPoseOnly* edge = new EdgeProjectionPoseOnly(pt3d_eigen, K_eigen);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(Eigen::Vector2d(pts2d_curr[idx].x, pts2d_curr[idx].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            optimizer.addEdge(edge);
            edges.push_back(edge);
        }

        // 优化，并剔除外点
        const double chi2_th = 5.991;
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        int g2o_inliers_count = 0;
        for (auto& edge : edges) {
            if (edge->chi2() > chi2_th) {
                edge->setLevel(1);
            } else {
                edge->setLevel(0);
                g2o_inliers_count++;
            }
        }

        if (g2o_inliers_count < 15) {
            return false;
        }
        
        // 5. 得到最终精化的相对位姿 T_c_l
        relative_pose = vertex_pose->estimate();
        
        // --- 可视化 ---
        // ... (visualization code remains the same) ...

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
        const int recent_kf_window = 12; 
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
        // 首先，保存所有关键帧优化前的位姿，用于计算位姿增量
        std::map<unsigned long, SE3> old_kf_poses;
        for (auto& kf_pair : all_kfs) {
            old_kf_poses[kf_pair.first] = kf_pair.second->Pose();
        }

        // 更新所有关键帧的位姿
        for (auto &v_pair : vertices) {
            all_kfs.at(v_pair.first)->SetPose(v_pair.second->estimate());
        }

        // [新增] 仿照ssvio，使用位姿增量来校正所有地图点
        Map::LandmarksType all_mpts = map_->GetAllMapPoints();
        for (auto& mpt_pair : all_mpts) {
            auto mpt = mpt_pair.second;
            if (mpt == nullptr || mpt->is_outlier_) continue;

            // 找到观测到该地图点的第一个关键帧作为参考帧
            auto observations = mpt->GetObs();
            if (observations.empty()) continue;
            auto ref_feat = observations.front().lock();
            if (!ref_feat) continue;
            auto ref_kf = ref_feat->frame_.lock();
            if (!ref_kf) continue;

            // 获取参考帧优化前后的位姿
            const SE3& T_ref_w_old = old_kf_poses.at(ref_kf->keyframe_id_);
            const SE3& T_ref_w_new = ref_kf->Pose(); // 已更新为优化后的位姿

            // 将地图点从旧世界坐标系转换到参考帧的局部坐标系
            Vec3 p_world_old = mpt->pos_;
            Vec3 p_local = T_ref_w_old * p_world_old;

            // 再从局部坐标系转换回新的世界坐标系，完成位置校正
            mpt->SetPos(T_ref_w_new.inverse() * p_local);
        }

        std::cout << "Full graph optimization finished." << std::endl;

        // 通知后端恢复
        if (backend_) {
            backend_->Resume();
        }
    }
}