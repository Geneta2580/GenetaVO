#include "myslam/frontend.h"

namespace myslam {
    
    // 前端处理
    Frontend::Frontend():current_frame_(std::make_shared<Frame>()), last_frame_(std::make_shared<Frame>()),
        camera_left_(std::make_shared<Camera>()), camera_right_(std::make_shared<Camera>()), map_(std::make_shared<Map>()),
        backend_(std::make_shared<Backend>()), loop_(std::make_shared<Loopclosing>()), viewer_(std::make_shared<Viewer>())
        {}  // 初始化成员变量，很重要，防止空指针影响初始化

    bool Frontend::Calculate(myslam::Frame::Ptr frame) {
        current_frame_ = frame;

        // --- 关键修改：移除全局地图锁 ---
        // --- 前端追踪不再需要持有地图锁，以实现与后端的并行 ---
        // {
        //     std::unique_lock<std::mutex> lock(map_->map_update_mutex_);
            switch (status_) {
                case FrontendStatus::INIT:
                    SteroInit(); // 进行数据初始化
                    break;
                case FrontendStatus::TRACKING_GOOD:
                case FrontendStatus::TRACKING_BAD:
                    Track(); // 进行光流追踪、位姿估计（必要时刻插入关键帧）
                    break;
                case FrontendStatus::LOST:
                    ReTrack();
                    break;
        }
        // --- 关键修复：移除这个多余的右花括号 ---
        // }

        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_); // 加入一个新帧，原本没有帧值的viewer线程从阻塞状态被激活
        }

        last_frame_ = current_frame_; // 变为过去帧
        return true;
    }
    
    bool Frontend::SteroInit() {
        loop_->SetBackend(backend_);
        backend_->SetLoopclosing(loop_); // 回环和后端线程相互连接，方便传输回环数据

        camera_left_->fx_ = myslam::Config::Get<double>("camera.fx"); // 设置左相机内参
        camera_left_->fy_ = myslam::Config::Get<double>("camera.fy");
        camera_left_->cx_ = myslam::Config::Get<double>("camera.cx");
        camera_left_->cy_ = myslam::Config::Get<double>("camera.cy");

        camera_right_->fx_ = myslam::Config::Get<double>("camera.fx"); // 设置右相机内参
        camera_right_->fy_ = myslam::Config::Get<double>("camera.fy");
        camera_right_->cx_ = myslam::Config::Get<double>("camera.cx");
        camera_right_->cy_ = myslam::Config::Get<double>("camera.cy");

        SE3 init_left_pose = Sophus::SE3d();
        Sophus::SE3d translation_transform(
            Sophus::SO3d(), 
            Eigen::Vector3d(-0.537166, 0.0, 0.0) // 左右目距离为0.537166
        );
        Sophus::SE3d init_right_pose = init_left_pose * translation_transform;

        DetectLeftFeatures(); // 进行左图中特征点检测
        FindRightFeatures(); // 进行右图中特征点的光流追踪

        camera_left_->pose_ = init_left_pose; // 设置初始帧左目图像的位姿为单位阵
        camera_right_->pose_ = init_right_pose;
        current_frame_->SetPose(init_left_pose);
        current_frame_->SetRelativePose(init_left_pose); // 设置当前帧的相对位姿为单位阵

        if (MapInit()) { // 地图初始化并且成功
            status_ = FrontendStatus::TRACKING_GOOD; // 切换前端工作状态码
            
            // 设置所有依赖
            loop_->SetMap(map_); // 将地图传输给回环
            backend_->SetCameras(camera_left_, camera_right_); // 将相机左右的位姿（实际上只要左右相对的位姿即可）赋给空指针
            backend_->SetMap(map_);
            viewer_->SetMap(map_);

            // 在所有依赖都设置好之后，再启动线程
            backend_->Start();
            viewer_->Start();

            InsertKeyFrame(); // 将第一帧作为关键帧插入

            return true;
        }

        return false;
    }

    int Frontend::DetectLeftFeatures() {
        // 使用GFTT描述子进行图像特征检测,特征点数量参数从yaml文件中加载      
        // cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 30); // 质量阈值，特征点最小间距，参数需要进行适当调整

        // 这个是FAST检测
        // cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(80); 
        // fast->setNonmaxSuppression(true);

        // 这个是ORB检测
        cv::Ptr<cv::ORB> orb = cv::ORB::create(num_features_*4, 1.2, 8);

        // --- 为保证BAD新检测的特征点不与之前的重合，增加掩膜 ---
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_) {
            if (feat) {
                cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                              feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
            }
        }

        // 对图像进行直方图均衡化增强，提升特征点检测效果
        cv::Mat enhanced_img;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(6.0, cv::Size(4, 4)); // 将图像分割为4x4的小块，每个小块内都进行直方图均衡化(阈值为6，越大对比度越强，噪声影响越大)
        clahe->apply(current_frame_->left_img_, enhanced_img);

        std::vector<cv::KeyPoint> all_keypoints;
        orb->detect(enhanced_img, all_keypoints, mask);

        // gftt->detect(current_frame_->left_img_, keypoints); // 将提取到的特征点注入到对应帧的类参数当中去
        // fast->detect(current_frame_->left_img_, keypoints); // FAST

        // --- 使用网格化筛选策略以保证特征点分布均匀 ---
        const int grid_size = 20; // 网格单元的像素大小
        const int grid_cols = current_frame_->left_img_.cols / grid_size;
        const int grid_rows = current_frame_->left_img_.rows / grid_size;

        // 创建网格，每个单元存储响应最强的关键点
        // 使用指针避免不必要的KeyPoint对象拷贝
        std::vector<std::vector<const cv::KeyPoint*>> grid(grid_rows, std::vector<const cv::KeyPoint*>(grid_cols, nullptr));

        for (const auto& kp : all_keypoints) {
            int row = static_cast<int>(kp.pt.y / grid_size);
            int col = static_cast<int>(kp.pt.x / grid_size);

            // 确保索引在有效范围内
            if (row >= 0 && row < grid_rows && col >= 0 && col < grid_cols) {
                if (grid[row][col] == nullptr || kp.response > grid[row][col]->response) {
                    grid[row][col] = &kp;
                }
            }
        }

        // 从网格中收集最终的关键点
        std::vector<cv::KeyPoint> final_keypoints;
        final_keypoints.reserve(grid_rows * grid_cols);
        for (int row = 0; row < grid_rows; ++row) {
            for (int col = 0; col < grid_cols; ++col) {
                if (grid[row][col] != nullptr) {
                    final_keypoints.push_back(*grid[row][col]);
                }
            }
        }
        // --- 网格化筛选结束 ---

        // 筛选后的均匀化关键点计算描述子
        cv::Mat descriptors; // ORB
        orb->compute(current_frame_->left_img_, final_keypoints, descriptors); // ORB
        current_frame_->SetDescriptors(descriptors); // 将ORB特征描述子进行传递
        
        int cnt_detected = 0;
        for(auto &kp: final_keypoints){
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected ++;
        }
        
        // std::cout << "Detect " << cnt_detected << " new features" << std::endl; // 输出检测的特征点数量
        return cnt_detected;
    }

    int Frontend::FindRightFeatures() {
        std::vector<cv::Point2f> kps_left, kps_right;
        for(auto &kp : current_frame_->features_left_) {  // 在非关键帧过程中这里的features_left_由上一帧追踪而来，所以有可能是空指针，注意判断
            if (kp) {
                kps_left.push_back(kp->position_.pt); // 注意指针形式，kp是一个feature类，要注入的是feature类中特征点的2D位置数据         
                auto mp = kp->map_point_.lock();
                if (mp) {
                    auto px = camera_right_->world2pixel(mp->pos_, current_frame_->RelativePose() * reference_frame_->Pose());
                    kps_right.push_back(cv::Point2f(px[0], px[1])); // 使用投影点作为初始值
                } else {   
                    kps_right.push_back(kp->position_.pt); // 使用左图中相同的坐标作为初始值
                }  
            }
        }

        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_,      // 利用光流法检测匹配左图的右图特征点,注意两张图必须灰度图,不能在之前显示彩点啥的
            kps_left, kps_right,    // 输入/输出特征点
            status, err,            // 跟踪状态，追到为1，没追到为0，和误差
            cv::Size(11, 11),       // 搜索窗口尺寸
            3,                      // 金字塔层数
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW // 使用初始值是否会更好？
        );
        
        int num_good_kps = 0;
        for(size_t i = 0;i < status.size(); i++) {
            if(status[i]) {
                cv::KeyPoint kp(kps_right[i], 7); // 创建一个大小为7的关键点
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false; // 标记特征点提在右图
                current_frame_->features_right_.push_back(feat);
                num_good_kps++;
            }
            else {
                current_frame_->features_right_.push_back(nullptr); // 若不能匹配，则为空，保持左右匹配点的vector对齐，方便查找匹配
            }
        }

        // std::cout << "Detect " << num_good_kps << " good features in right frame" << std::endl; // 输出在右目图像中可追踪到的特征点数量
        return num_good_kps;
    }

    int Frontend::LKDetectLastFeatures() {
        std::vector<cv::Point2f> kps_last, kps_current;
        std::vector<Feature::Ptr> last_features_tracked; // 上一帧追踪的特征点

        for (auto &feat : last_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (feat && mp) { // 跳过空指针, 并且只追踪已三角化的点
                last_features_tracked.push_back(feat); // 保存特征点
                // T_c_w = T_c_r * T_r_w
                auto px = camera_left_->world2pixel(mp->pos_, current_frame_->RelativePose() * reference_frame_->Pose());
                kps_last.push_back(feat->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1])); // 该特征点有关联的地图点，使用重投影作为初始估计
            }
            // else if (feat) { // 如果没有关联地图点，则使用上一帧的特征点位置作为初始估计   
            //     kps_current.push_back(feat->position_.pt); // 使用上一帧的特征点位置作为初始估计
            // }
        }

        if (kps_last.empty()) {
            return 0; // 没有可追踪的点？
        }

        // Step 2: 使用光流法进行追踪
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_,
            kps_last, kps_current,
            status, err,
            cv::Size(11, 11),
            3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW
        );

        // Step 3: 为当前帧创建新的特征点，并继承地图点关联
        int num_good_kps = 0;
        for(size_t i = 0; i < status.size(); i++) {
            if(status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_features_tracked[i]->map_point_;  //从保存的 last_features_tracked 中获取对应的地图点
                current_frame_->features_left_.push_back(feature);
                num_good_kps++;
            }
        }

        // std::cout << "Detect " << num_good_kps << " good features in last frame" << std::endl;
        return num_good_kps;
    }

    int Frontend::BACurrentPose() {
        // 构建图优化，先设定g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        // 对于小规模的Pose-Only BA，使用稠密求解器在数值上更稳定
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg( // 设置优化算法，求解器
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
    
        // 顶点设置
        VertexPose *vertex_pose = new VertexPose();  // 相机位姿顶点
        vertex_pose->setId(0);
        // 初始值是 T_cw = T_cr * T_rw
        vertex_pose->setEstimate(current_frame_->RelativePose() * reference_frame_->Pose());
        optimizer.addVertex(vertex_pose); // 加入顶点
    
        // 相机内参矩阵
        Mat33 K = camera_left_->K();

        // 边设置
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges; // 自带一元边，仅优化相机位姿，把3D点坐标作为固定参数
        std::vector<Feature::Ptr> features;

        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {  // 有多少特征点就有多少条边，一个位姿顶点对n个特征点
            if (current_frame_->features_left_[i] == nullptr) {
                continue; // 跳过空指针
            }
            auto mp = current_frame_->features_left_[i]->map_point_.lock(); // 若存在三角化后的地图点
            
            if (mp && !mp->is_outlier_) {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K); // 传入世界坐标以及内参矩阵 
                edge->setId(index); // 注意这里的index要和顶点id错开
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt)); // 把特征点的两个坐标x\y转换为二维量测信息?
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber); // 使用Huber鲁棒核函数
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }
    
        // --- 最终修复：在优化前检查是否有边被添加 ---
        if (optimizer.edges().empty()) {
            // 如果没有边，意味着当前帧没有观测到任何地图点，优化无意义
            return 0;
        }

        // // 添加顶点和重投影误差边
        // if (last_frame_) {
        //     VertexPose* last_vertex_pose = new VertexPose();
        //     last_vertex_pose->setId(1); // 新的顶点
        //     last_vertex_pose->setEstimate(last_frame_->Pose());
        //     last_vertex_pose->setFixed(true); // 固定上一帧的位姿
        //     optimizer.addVertex(last_vertex_pose);
        // }

        // 开始进行位姿的BA优化
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; iteration++) {   // 进行4次迭代，每次迭代优化10次
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;
    
            // 判定优化中的异常值
            for (size_t i = 0; i < edges.size(); ++i) {
                auto e = edges[i];
                if (features[i]->is_outlier_) {
                    e->computeError();
                }
                if (e->chi2() > chi2_th) { // 计算误差大于阈值，判断为异常值，采用分层机制，分层进行优化
                    features[i]->is_outlier_ = true;
                    e->setLevel(1); // 优化层级降为1
                    cnt_outlier++; // 异常值计数
                } else {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0); // 优化层级保持为0，优先进行
                };
    
                if (iteration == 2) { // 迭代值为2禁用鲁棒核函数？
                    e->setRobustKernel(nullptr);
                }
            }
        }
    
        // std::cout << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
        //           << features.size() - cnt_outlier << std::endl;

        // 优化结果输出到当前帧的位姿
        current_frame_->SetPose(vertex_pose->estimate());
        // 反向计算出 T_cr = T_cw * T_rw^-1
        current_frame_->SetRelativePose(vertex_pose->estimate() * reference_frame_->Pose().inverse());
    
        // std::cout << "Current Pose = \n" << current_frame_->Pose().matrix() << std::endl  << std::endl;
    
        for (auto &feat : features) {
            if (feat->is_outlier_) {
                MapPoint::Ptr mp = feat->map_point_.lock();
                if (mp && current_frame_->id_ - reference_frame_->id_ <= 2) {
                    mp->is_outlier_ = true;
                    map_->AddOutlierMapPoint(mp->id_);
                }
                feat->map_point_.reset(); // 断开与地图点的关联
                feat->is_outlier_ = false;  // 重置标记
            }
        }
        return features.size() - cnt_outlier; // 返回剔除异常点的计数
    }

    int Frontend::Triangulation() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        int cnt_triangulated_pts = 0;
        // T_w_c = (T_c_r * T_r_w)^-1
        SE3 current_pose_Twc = (current_frame_->RelativePose() * reference_frame_->Pose()).inverse();

        // std::cout << poses[0].matrix() << std::endl; // 测试用查看传入的位姿矩阵

        for (size_t i = 0; i < current_frame_->features_left_.size(); i++) { // 这里左右无所谓，因为size是一样的
            if (current_frame_->features_left_[i]->map_point_.lock()) {
                continue;
            }
            if (current_frame_->features_right_[i] == nullptr) continue;  // 保证右图匹配点存在
            std::vector<Vec3> points {                                        
                camera_left_->pixel2camera(                                   // 像素点转换为归一化平面（相机）点，便于三角化
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                        current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                            current_frame_->features_right_[i]->position_.pt.y))};

            Vec3 pt_world = Vec3::Zero();

            if (triangulation(poses, points, pt_world) && pt_world[2] > 0) { // 进行三角化，同时判断三角化后三维点数值的有效性
                auto new_map_point = MapPoint::CreateNewMappoint();
                // 从相机坐标系转换到世界坐标系
                pt_world = current_pose_Twc * pt_world;
                new_map_point->SetPos(pt_world);     // 设置地图点

                map_->InsertMapPoint(new_map_point); // 将地图点插入地图
                viewer_->InsertNewMapPoint(new_map_point); // 将地图点插入可视化界面

                current_frame_->features_left_[i]->map_point_ = new_map_point; // 地图点和左右图像特征点匹配
                current_frame_->features_right_[i]->map_point_ = new_map_point;

                cnt_triangulated_pts++; // 三角化成功
            }

        }
        
        // std::cout << "Triangulated successful with " << cnt_triangulated_pts
        //           << " map points" << std::endl;

        return cnt_triangulated_pts;
    }

    bool Frontend::MapInit() {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i] == nullptr) continue; // 跳过不能追踪到右图特征点的情况

            // 三角化创建初始新地图
            std::vector<Vec3> points{           // 像素点转换为归一化平面（相机）点，便于三角化
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            
            Vec3 pworld = Vec3::Zero();
    
            if (triangulation(poses, points, pworld) && pworld[2] > 0) // 三角化并判断是否有效
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]); // 将左图特征点存储到对应的地图点观测当中
                new_map_point->AddObservation(current_frame_->features_right_[i]);

                viewer_->InsertNewMapPoint(new_map_point); // 将地图点插入可视化界面

                current_frame_->features_left_[i]->map_point_ = new_map_point;  // 地图点和左右图像特征点匹配
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
        }
    
        std::cout << "Initial map created with " << cnt_init_landmarks
                  << " map points" << std::endl;
        
        if (cnt_init_landmarks > 50) {
            return true;
        }
        return false;
    }

    int Frontend::Track() {
        // 恒速模型预测 T_cr_new = T_c-1,c * T_c-1,r
        if (last_frame_) {
            current_frame_->SetRelativePose(relative_motion_ * last_frame_->RelativePose());
        }
        
        LKDetectLastFeatures(); // 光流法追踪特征点进行匹配

        num_track_good_ = BACurrentPose(); // BA优化当前位姿，并返回内点数量

        if (num_track_good_ > num_features_tracking_good_) {
            // std::cout << "Tracking good: " << num_features_tracking_good_ << std::endl;
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (num_track_good_ > num_features_tracking_bad_) {
            // std::cout << "Tracking bad." << std::endl;
            status_ = FrontendStatus::TRACKING_BAD;
        } else {
            status_ = FrontendStatus::LOST;
            std::cout << "Tracking lost, try to relocalize or reinitialize." << std::endl;
        }

        // 注意，在BAD之前更新
        if (status_ != FrontendStatus::LOST) {
            relative_motion_ = current_frame_->RelativePose() * last_frame_->RelativePose().inverse();
        }
        // 注意，在BAD之前更新

        if (status_ == FrontendStatus::TRACKING_BAD) {
            // “补充”策略：不清除现有特征点，而是检测新的来补充
            DetectLeftFeatures();
            FindRightFeatures();
            Triangulation();
            InsertKeyFrame();
        } else if (status_ == FrontendStatus::LOST) {
            // “重置”策略
            ReTrack();
        } else { // TRACKING_GOOD
            // 追踪良好，根据运动量判断是否插入关键帧
            // if (relative_motion_.log().norm() > 0.05) { // 阈值可调
            //     InsertKeyFrame();
            // }
        }

        return num_track_good_;
    }

    void Frontend::ReTrack() {
        // 清空之前优化不好的特征点
        std::vector<std::shared_ptr<Feature>>().swap(current_frame_->features_left_);

        DetectLeftFeatures(); // 重新进行GFTT检测
        FindRightFeatures();
        Triangulation(); // 对关键帧特征点进行三角化
        InsertKeyFrame(); // 插入关键帧

        // std::cout << "keyframe id:" << current_frame_->keyframe_id_ << std::endl; // 测试用检查关键帧id
        // std::cout << "frame id:" << current_frame_->id_ << std::endl; // 测试用检查帧id

        status_ = FrontendStatus::TRACKING_GOOD;
    }

    void Frontend::InsertKeyFrame() {
        
        current_frame_->SetKeyFrame();
        
        // 关键：计算并设置新关键帧的绝对位姿
        if (reference_frame_) {
            current_frame_->SetPose(current_frame_->RelativePose() * reference_frame_->Pose());
        }
        
        // --- 而是将其放入后端的处理队列中 ---
        backend_->InsertNewKeyFrame(current_frame_);

        if (viewer_) {
            viewer_->UpdateMap();
        }

        reference_frame_ = current_frame_;
        // 新的参考帧，其相对位姿是单位矩阵
        current_frame_->SetRelativePose(SE3());
    }

    void Frontend::Stop() {
        backend_->Stop();
        loop_->Stop();
        viewer_->Close();
    }
}