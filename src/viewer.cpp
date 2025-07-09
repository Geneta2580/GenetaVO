#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

    Viewer::Viewer() {
        viewer_running_ = true;
        // 线程的启动被移到 Start() 函数
    }

    void Viewer::Start() {
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    }

    void Viewer::Close() { // 关闭线程
        viewer_running_ = false;
        if (viewer_thread_.joinable()) {
            viewer_thread_.join();
        }
    }

    void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
        std::lock_guard<std::mutex> lck(frame_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap() {
        // 这个函数现在假定调用者（Frontend）已经持有了地图的大锁
        // std::cout << "Updating map in viewer..." << std::endl;
        assert(map_ != nullptr);
        std::unique_lock<std::mutex> lck(map_->map_update_mutex_);
        all_frames_ = map_->GetAllKeyFrames();
        all_landmarks_ = map_->GetAllMapPoints();
        map_updated_ = true;
    }

    void Viewer::ThreadLoop() {
        pangolin::CreateWindowAndBind("GenetaSLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        pangolin::View& vis_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        const float green[3] = {0, 1, 0};

        while (!pangolin::ShouldQuit() && viewer_running_) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            Frame::Ptr local_frame;
            {
                std::lock_guard<std::mutex> lck(frame_mutex_);
                local_frame = current_frame_;
            }

            if (local_frame) {
                DrawFrame(local_frame, green);
                // FollowCurrentFrame(vis_camera, local_frame);
                PlotFrameImage(local_frame);
                cv::waitKey(1);
            }

            {
                std::unique_lock<std::mutex> map_lock(map_->map_update_mutex_);
                if (map_) {
                    DrawMapPoints();
                }
                
                if (local_frame) {
                    trajectory_.push_back(local_frame->Pose().inverse().translation());
                }
                if (!trajectory_.empty()) {
                    DrawTrajectory();
                }
            } // map_lock 在这里自动释放

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
        
        for (auto& af : all_frames_) {
            DrawFrame(af.second, blue);
        }

        // glPointSize(2);
        // glBegin(GL_POINTS);
        // for (auto& landmark : all_landmarks_) {
        //     auto pos = landmark.second->Pos();
        //     glColor3f(red[0], red[1], red[2]);
        //     glVertex3d(pos[0], pos[1], pos[2]);
        // }
        // glEnd();
    }

    void Viewer::DrawTrajectory() {
        const float green[3] = {0, 1, 0};
        const float line_width = 2.0f;
    
        glLineWidth(line_width);
        glBegin(GL_LINE_STRIP);
        glColor3fv(green);
        
        for (const auto& pos : trajectory_) {
            glVertex3f(pos[0], pos[1], pos[2]);
        }
        
        glEnd();
    }
}
