#include "myslam/backend.h"
#include "myslam/algorithm.h"
#include "myslam/feature.h"
#include "myslam/g2o_param.h"
#include "myslam/map.h"
#include "myslam/mappoint.h"

namespace myslam {

    Backend::Backend() {
        backend_running_.store(true);
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this)); // 启动后端优化线程
    }
    
    void Backend::RequestPause() {
        pause_requested_.store(true);
        map_update_.notify_one(); // 立刻让后端暂停，防止被前端更新影响
    }

    void Backend::Resume() {
        pause_requested_.store(false);
        resume_cv_.notify_one();
    }

    void Backend::Wake() {
        map_update_.notify_one(); // 直接通知，无需加锁
    }
    
    void Backend::Stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        Resume(); // 唤醒可能处于暂停等待的线程
        backend_thread_.join();
    }

    void Backend::BackendLoop() { // 后端优化主线程
        while (backend_running_.load()) {
            {
                std::unique_lock<std::mutex> lock(data_mutex_);
                map_update_.wait(lock);
            }

            // 在退出循环前，再次检查运行状态
            if (!backend_running_.load()) {
                break;
            }

            // 暂停检查点：如果收到暂停请求，则在此等待
            {
                std::unique_lock<std::mutex> pause_lock(pause_mutex_);
                resume_cv_.wait(pause_lock, [this] { return !pause_requested_.load(); }); // pause_requested_为false则继续执行
            }

            // 在暂停后再次检查，因为线程可能在Stop()中被唤醒
            if (!backend_running_.load()) {
                break;
            }

            // 获取地图大锁，准备执行优化
            std::unique_lock<std::mutex> map_lock(map_->map_update_mutex_);

            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            
            // 优化触发条件：地图中的关键帧数量需要足够多
            if (active_kfs.size() < 2) {
                continue; 
            }

            // Optimize(active_kfs, active_landmarks); // 优化函数
        }
    }

    void Backend::Optimize(Map::KeyframesType &keyframes,   // 优化后的路标点和关键帧位姿返回到变量，实现对输入的操作
                           Map::LandmarksType &landmarks) {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType; // 必须使用稀疏求解器

        auto solver = new g2o::OptimizationAlgorithmLevenberg( // 设置优化算法，求解器
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
    
        // pose 顶点，使用Keyframe id
        std::map<unsigned long, VertexPose *> vertices; // 位姿顶点键值对
        unsigned long max_kf_id = 0;

        for (auto &keyframe : keyframes) {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();  // 相机位姿顶点
            vertex_pose->setId(kf->keyframe_id_); // 顶点id为关键帧id
            vertex_pose->setEstimate(kf->Pose()); // 设定关键帧位姿优化初值
            optimizer.addVertex(vertex_pose); // 加入位姿顶点
            if (kf->keyframe_id_ > max_kf_id) {
                max_kf_id = kf->keyframe_id_;
            }
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // 路标顶点，使用路标id索引
        std::map<unsigned long, VertexXYZ *> vertices_landmarks; // 路标顶点键值对
    
        // K和左右外参
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();
    
        // edges
        int index = 1;
        double chi2_th = 5.991;  // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;
        for (auto &landmark : landmarks) {
            if (landmark.second->is_outlier_) continue; // 当BA解出的路标越界
            unsigned long landmark_id = landmark.second->id_;
            auto observations = landmark.second->GetActiveObs(); // 获取活跃观测

            // 如果landmark还没有被加入优化，则新加一个顶点
            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1); 
                v->setMarginalized(true); 
                
                // 关键改动：采用ssvio的锚点策略，并进行严格的空指针检查
                if (!observations.empty()) {
                    auto feat = observations.front().lock();
                    if (feat) {
                        auto frame = feat->frame_.lock();
                        if (frame && keyframes.find(frame->keyframe_id_) == keyframes.end()) {
                            v->setFixed(true);
                        }
                    }
                }

                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }

            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue; // 特征点非空（右侧的提取的时候为了对齐有可能为空）
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
    
                auto frame = feat->frame_.lock(); 
                if (vertices.find(frame->keyframe_id_) == vertices.end()) continue;

                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) { 
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }
    
                edge->setId(index);
                edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                edge->setMeasurement(toVec2(feat->position_.pt));
                edge->setInformation(Mat22::Identity());
                auto rk = new g2o::RobustKernelHuber(); 
                rk->setDelta(chi2_th); 
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge, feat});
                optimizer.addEdge(edge);
                index++;
            }
        }

        if (edges_and_features.empty()) {
            // 如果没有边，意味着当前活跃帧无法观测到任何活跃地图点，优化无意义
            return;
        }
    
        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            optimizer.initializeOptimization();
            optimizer.optimize(10); 
            cnt_outlier = 0;
            cnt_inlier = 0;
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) { // edge的优化误差大于阈值
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.7) { // 优化良好的点的比例大于50%，直接结束优化
                break;
            } else { // 优化良好的点的比例小于50%，调整鲁棒核函数阈值 
                // chi2_th *= 2;
                iteration++;
            }
        }
    
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                auto mappoint = ef.second->map_point_.lock();
                if (mappoint) { // <-- 关键的非空检查
                    mappoint->RemoveActiveObservation(ef.second); // 优化误差大，去除活跃观测
                    mappoint->RemoveObservation(ef.second); // 同时移除总观测

                    if (mappoint->GetObs().empty()) { 
                        mappoint->is_outlier_ = true;
                        map_->AddOutlierMapPoint(mappoint->id_);
                    }
                }
                ef.second->map_point_.reset(); // 断开与地图点的关联
            } else {
                ef.second->is_outlier_ = false;
            }
        }
    
        for (auto &v : vertices) {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }

        // delete outlier mappoints
        map_->RemoveAllOutlierMapPoints();
        map_->RemoveOldActiveMapPoints();
    }
    
}