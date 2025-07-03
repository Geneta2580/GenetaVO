#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

// algorithms used in myslam
#include "opencv2/core/eigen.hpp"
#include "myslam/common.h"
#include "myslam/config.h"

namespace myslam {

    /**
     * linear triangulation with SVD
     * @param poses     相机位姿，比较重要，若使用归一化坐标，对于左右目图像的poses相对值有要求，而对绝对值没有要求
     * @param points    这里的points输入是相机归一化平面上的坐标
     * @param pt_world  三角化后的左目归一化坐标地图点
     * @return true if success
     */
    inline bool triangulation(const std::vector<SE3> &poses, 
        const std::vector<Vec3> points, Vec3 &pt_world) {
            // 初始化世界3D坐标
            float x = 0.0f, y= 0.0f, z= 0.0f;

            // // SE3转换为cv::Mat
            // cv::Mat R_left_cv, t_left_cv, R_right_cv, t_right_cv;
            // Eigen::Matrix3d R_left = poses[0].rotationMatrix();
            // Eigen::Vector3d t_left = poses[0].translation();
            // Eigen::Matrix3d R_right = poses[1].rotationMatrix();
            // Eigen::Vector3d t_right = poses[1].translation();

            // cv::eigen2cv(R_left, R_left_cv); // 默认类型为 CV_64FC1
            // cv::eigen2cv(t_left, t_left_cv);
            // cv::eigen2cv(R_right, R_right_cv);
            // cv::eigen2cv(t_right, t_right_cv);
            
            // cv::Mat extrinsic_left = cv::Mat::zeros(3, 4, CV_64FC1); // 构造外参矩阵 [R | t]
            // cv::Mat extrinsic_right = cv::Mat::zeros(3, 4, CV_64FC1);
            // R_left_cv.copyTo(extrinsic_left(cv::Rect(0, 0, 3, 3))); // 填充 R
            // t_left_cv.copyTo(extrinsic_left(cv::Rect(3, 0, 1, 3))); // 填充 t
            // R_right_cv.copyTo(extrinsic_right(cv::Rect(0, 0, 3, 3))); // 填充 R
            // t_right_cv.copyTo(extrinsic_right(cv::Rect(3, 0, 1, 3))); // 填充 t

            // std::vector<cv::Point2f> left_pts, right_pts;
            // left_pts.emplace_back(points[0].x(), points[0].y());  // 取前两维作为二维坐标
            // right_pts.emplace_back(points[1].x(), points[1].y());


            // 转换位姿为 T_cw（假设输入的poses是T_wc）这个是opencv函数的需要
            Eigen::Matrix3d R_wc0 = poses[0].rotationMatrix();
            Eigen::Vector3d t_wc0 = poses[0].translation();
            Eigen::Matrix3d R_cw0 = R_wc0.transpose();
            Eigen::Vector3d t_cw0 = -R_cw0 * t_wc0;

            Eigen::Matrix3d R_wc1 = poses[1].rotationMatrix();
            Eigen::Vector3d t_wc1 = poses[1].translation();
            Eigen::Matrix3d R_cw1 = R_wc1.transpose();
            Eigen::Vector3d t_cw1 = -R_cw1 * t_wc1;

            // 转换为 OpenCV 的外参矩阵 [R | t]
            cv::Mat R_left, t_left, R_right, t_right;
            cv::eigen2cv(R_cw0, R_left);
            cv::eigen2cv(t_cw0, t_left);
            cv::eigen2cv(R_cw1, R_right);
            cv::eigen2cv(t_cw1, t_right);

            cv::Mat extrinsic_left = cv::Mat::zeros(3, 4, CV_64FC1);
            cv::Mat extrinsic_right = cv::Mat::zeros(3, 4, CV_64FC1);
            R_left.copyTo(extrinsic_left(cv::Rect(0, 0, 3, 3)));
            t_left.copyTo(extrinsic_left(cv::Rect(3, 0, 1, 3)));
            R_right.copyTo(extrinsic_right(cv::Rect(0, 0, 3, 3)));
            t_right.copyTo(extrinsic_right(cv::Rect(3, 0, 1, 3)));

            // 输入点应为归一化坐标 (x, y, 1)，取前两维
            std::vector<cv::Point2f> left_pts, right_pts;
            left_pts.emplace_back(points[0].x(), points[0].y());
            right_pts.emplace_back(points[1].x(), points[1].y());

            // 三角化
            cv::Mat pts_4d;
            cv::triangulatePoints(extrinsic_left, extrinsic_right, left_pts, right_pts, pts_4d); // 这里输入的是归一化坐标，所以不需要输入K了

            cv::Mat col = pts_4d.col(0);

            float w = col.at<float>(3, 0);
           
            // std::cout << "w:" << w << std::endl; // 测试，w大小

            if (w != 0 && std::abs(w) > 1e-6) {  // 保证不会溢出同时避免除零情况
                x = col.at<float>(0, 0) / w;
                y = col.at<float>(1, 0) / w;
                z = col.at<float>(2, 0) / w;

                // std::cout << "xyz: " << x << ","<< y << ","<< z << std::endl; // 测试，xyz大小
            }
            
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z >= 0.0f) { // 保证xyz有效性
                pt_world << static_cast<double>(x),  // 提取并转换 x
                static_cast<double>(y),  // 提取并转换 y
                static_cast<double>(z);  // 提取并转换 z
                
                return true;
            }

            // std::cout << "Triangulation failed" << std::endl;
            return false;  // 可以包含多个return，先执行哪个后面就不执行
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

}

#endif
