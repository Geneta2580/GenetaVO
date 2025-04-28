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
    
    void Backend::Wake() {
        std::unique_lock<std::mutex> lock(data_mutex_); // 需要互斥锁
        map_update_.notify_one(); // 唤醒线程
    }
    
    void Backend::Stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }
    
    void Backend::BackendLoop() { // 后端优化主线程
        while (backend_running_.load()) {
            std::unique_lock<std::mutex> lock(data_mutex_); // 等待线程被唤醒
            map_update_.wait(lock);

            map_->InsertKeyFrame(new_key_frame_);
            map_->InsertMapPoint(new_map_point_);

            /// 后端优化关键帧和地图路标点
            Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
            
            Optimize(active_kfs, active_landmarks); // 优化函数
        }
    }
    
    void Backend::Optimize(Map::KeyframesType &keyframes,   // 优化后的路标点和关键帧位姿返回到变量，实现对输入的操作
                           Map::LandmarksType &landmarks) {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型

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
            auto observations = landmark.second->GetObs(); // 获取路标点对应的左右特征点值
            for (auto &obs : observations) {
                if (obs.lock() == nullptr) continue; // 特征点非空（右侧的提取的时候为了对齐有可能为空）
                auto feat = obs.lock();
                if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
    
                auto frame = feat->frame_.lock(); // 持有该特征的帧
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_image_) { // 若提在左图
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }
    
                // 如果landmark还没有被加入优化，则新加一个顶点
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end()) {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1); // 注意把位姿顶点和路标顶点的id分开
                    v->setMarginalized(true); // 路标顶点优化边缘化，必须边缘化，否则巨多路标点一起优化，直接爆炸
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }
    
                // 若路标已被加入优化（能在列表中找到）
                if (vertices.find(frame->keyframe_id_) != vertices.end() && 
                    vertices_landmarks.find(landmark_id) != vertices_landmarks.end()) {
                        edge->setId(index);
                        edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                        edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                        edge->setMeasurement(toVec2(feat->position_.pt));
                        edge->setInformation(Mat22::Identity());
                        auto rk = new g2o::RobustKernelHuber(); // 设置鲁棒核函数
                        rk->setDelta(chi2_th); // 阈值
                        edge->setRobustKernel(rk);
                        edges_and_features.insert({edge, feat});
                        optimizer.addEdge(edge);
                        index++;
                    }
                else delete edge;
                    
            }
        }

        // // 添加回环边
        // if (loop_->loop_id_.size() > last_loop_size_) {
        //     // for (const auto& [key, value] : loop_->loop_id_) {
        //     //     std::cout << "curr id:" << key << " value: " << value << std::endl;  // 测试用
        //     // }
        //     auto last_loop = loop_->loop_id_.rbegin(); // 指向最后一个元素的反向迭代器
        //     LoopEdge *edge = new LoopEdge;
        //     edge->setVertex(0, vertices.at(last_loop->first));
        //     edge->setVertex(1, vertices.at(last_loop->second));
        //     edge->setInformation(Mat66::Identity());
        //     optimizer.addEdge(edge);

        //     last_loop_size_ = loop_->loop_id_.size();
        // }

        // do optimization and eliminate the outliers
        optimizer.initializeOptimization();
        optimizer.optimize(10);
    
        // 开头5轮优化自适应调整鲁棒值
        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) { // edge的优化误差大于阈值
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) { // 优化良好的点的比例大于50%，不调整鲁棒核函数阈值
                break;
            } else { // 优化良好的点的比例小于50%，调整鲁棒核函数阈值 
                chi2_th *= 2;
                iteration++;
            }
        }
    
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                ef.second->is_outlier_ = true;
                ef.second->map_point_.lock()->RemoveObservation(ef.second); // 优化误差大，去除观测
            } else {
                ef.second->is_outlier_ = false;
            }
        }
    
        // std::cout << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
        //           << cnt_inlier << std::endl;
    
        // 给关键帧和路标点的位姿变量赋值，似乎不能可视化了，因为只有当前帧的位姿会被显示出来
        for (auto &v : vertices) {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v : vertices_landmarks) {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }
    
}