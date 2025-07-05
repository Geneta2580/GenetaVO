#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "myslam/common.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

    /**
     * 可视化
     */
    class Viewer {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }

        void Close();

        // 增加一个当前帧
        void AddCurrentFrame(Frame::Ptr current_frame);

        // 后端地图插入一个新地图点
        void InsertNewMapPoint(MapPoint::Ptr new_map_point) { new_map_point_ = new_map_point; }

        // 后端地图插入一个新关键帧
        void InsertNewKeyFrame(Frame::Ptr new_key_frame) { new_key_frame_ = new_key_frame; }

        // 更新地图
        void UpdateMap();

    private:
        
        // 绘图主线程
        void ThreadLoop();

        // pangolin绘制相机
        void DrawFrame(Frame::Ptr frame, const float* color); 

        // 绘制地图点
        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera, Frame::Ptr local_frame);

        // 画出当前帧的图像和特征点
        void PlotFrameImage(Frame::Ptr local_frame);

        // 画出轨迹
        void DrawTrajectory();

        // 调试用
        void DrawStaticLine(); 

        Frame::Ptr current_frame_ = nullptr; // 初始帧为空指针
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        // 激活的地图变量
        std::map<unsigned long, Frame::Ptr> active_frames_;
        std::map<unsigned long, MapPoint::Ptr> active_landmarks_;

        // 所有地图变量
        std::map<unsigned long, Frame::Ptr> all_frames_;
        std::map<unsigned long, MapPoint::Ptr> all_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;
        std::mutex frame_mutex_;  // 仅保护 current_frame_

        // 新帧的位姿数据
        SE3 Twc;
        std::vector<Vec3> trajectory_;      // 存储轨迹点

        // 插入的新帧、地图点
        Frame::Ptr new_key_frame_;
        MapPoint::Ptr new_map_point_;

    };
}

#endif
