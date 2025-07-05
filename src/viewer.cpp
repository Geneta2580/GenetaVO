#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

    Viewer::Viewer() {
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this)); // 创建线程
    }

    void Viewer::Close() { // 关闭线程
        viewer_running_ = false;
        viewer_thread_.join();
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {   // 初始帧为空指针，只有在外部传入值后current_frame才有值
        std::lock_guard<std::mutex> lck(frame_mutex_);  // 仅短暂锁定帧更新
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap() {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);
        all_frames_ = map_->GetAllKeyFrames();
        all_landmarks_ = map_->GetAllMapPoints();
        map_updated_ = true;
    }

    void Viewer::ThreadLoop() {  // 画图viewer可视化界面主线程，读程序从这里开始读
        
        pangolin::CreateWindowAndBind("GenetaSLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& vis_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        const float green[3] = {0, 1, 0};

        while (!pangolin::ShouldQuit() && viewer_running_) { // 判定viewer仍在工作                
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            Frame::Ptr local_frame;
            {
                std::lock_guard<std::mutex> lck(frame_mutex_);
                local_frame = current_frame_;  // 快速拷贝
            }  // 立即释放锁，后续渲染不再持有锁

            // DrawStaticLine(); // 测试用
            std::unique_lock<std::mutex> lock(viewer_data_mutex_);
            if (local_frame) { // 若从AddCurrentFrame传入了帧图像
                Twc = local_frame->Pose().inverse(); // 存储当前帧的位姿
                trajectory_.push_back(Twc.translation());
                if(local_frame->is_keyframe_){ // 若当前帧为关键帧，则更新
                    map_->InsertKeyFrame(new_key_frame_);
                    map_->InsertMapPoint(new_map_point_); // 此句必须在三角化执行之后，也就是说关键帧必三角化
                    // assert(map_ != nullptr);
                    all_frames_ = map_->GetAllKeyFrames();
                    all_landmarks_ = map_->GetAllMapPoints();
                    map_updated_ = true;
                }

                DrawFrame(local_frame, green); // 在pangolin中进行当前帧的绘制（像是一个画当前相机运动的）
                // FollowCurrentFrame(vis_camera); // 引入了当前帧的位姿变换

                PlotFrameImage(local_frame); // 在opencv中进行当前帧的绘制
                
                cv::waitKey(1); // 等待1ms
            }

            if (map_) { // 地图有效
                DrawMapPoints(); // 绘制地图点
            }

            if (!trajectory_.empty()) {
                DrawTrajectory();
            }

            pangolin::FinishFrame();
            usleep(5000);
        }

        std::cout << "Stop viewer" << std::endl;
    }

    void Viewer::PlotFrameImage(Frame::Ptr local_frame) {
        cv::Mat img_out1;
        cv::cvtColor(local_frame->left_img_, img_out1, cv::COLOR_GRAY2BGR); // 灰度图转换为BGR图
        
        for (size_t i = 0; i < local_frame->features_left_.size(); i++) { // 注意追踪上一帧得到结果没有nullptr
            if (local_frame->features_left_[i]->map_point_.lock()) { // 只能画左侧图像，右侧图像没法画，因为非关键帧不检测右图像
                auto &feat1 = *local_frame->features_left_[i];
                cv::circle(img_out1, feat1.position_.pt, 2, cv::Scalar(0, 250, 0), 2);
            }
        }

        cv::imshow("SLAM Left Frame Preview", img_out1);
    }

    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera, Frame::Ptr local_frame) {
        SE3 Twc = local_frame->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
        SE3 Twc = frame->Pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]); // 设定指定颜色

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void Viewer::DrawMapPoints() {
        const float red[3] = {1.0, 0, 0};
        const float blue[3] = {0, 0, 1.0};
        for (auto& af : all_frames_) {  // 画出所有关键帧
            DrawFrame(af.second, blue); // 这里all_frames_是一个键对值，std::pair，second表示输出pair的第二个索引
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto& landmark : all_landmarks_) {
            auto pos = landmark.second->Pos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

    // 渲染一条直线，与外部参数无关，测试多线程并行时用
    void Viewer::DrawStaticLine() {
        const float red[3] = {1.0, 0, 0}; // 红色直线
        const float line_width = 3.0f;
        
        glLineWidth(line_width);
        glBegin(GL_LINES);
        glColor3fv(red);
        // 世界坐标系下绘制X轴方向直线
        glVertex3f(0, 0, 0);   // 起点坐标
        glVertex3f(5, 0, 0);   // 终点坐标
        glEnd();
    }

    void Viewer::DrawTrajectory() {
        const float green[3] = {0, 1, 0};
        const float line_width = 2.0f;
    
        glLineWidth(line_width);
        glBegin(GL_LINE_STRIP); // 连续线段模式
        glColor3fv(green);
        
        for (const auto& pos : trajectory_) {
            glVertex3f(pos[0], pos[1], pos[2]); // 按顺序连接轨迹点
        }
        
        glEnd();
    }
}
