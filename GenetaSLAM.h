#ifndef MYSLAM_GENETA_SLAM_H
#define MYSLAM_GENETA_SLAM_H

#include "myslam/common.h"
#include "myslam/dataset.h"
#include <opencv2/highgui.hpp>
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/config.h"
#include "myslam/backend.h"

namespace myslam {

    // 再封装一层
    class GenetaSLAM {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            typedef std::shared_ptr<GenetaSLAM> Ptr;

            GenetaSLAM(const std::string &config_path);

            // 初始化
            bool Init();

            // 运行
            void Run();

            // 迭代
            bool Step();

        private:
            std::string config_file_path_;
            
            Frontend::Ptr frontend_ = nullptr;
            Backend::Ptr backend_ = nullptr;
            Viewer::Ptr viewer_ = nullptr;
        
            // dataset
            Dataset::Ptr dataset_ = nullptr;


    };
}

#endif