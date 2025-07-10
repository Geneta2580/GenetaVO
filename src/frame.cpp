#include "myslam/frame.h"

namespace myslam {
    Frame::Frame(long id, double timestamp, const Mat &left, const Mat &right) :
        id_(id), left_img_(left), right_img_(right) {}

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0; // 静态变量，每次进入都不一样
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++; // 分配ID
        return new_frame;
    }

    void Frame::SetKeyFrame() {
        static long keyframe_factory_id = 0; // 设置关键帧工厂的ID
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

    // 拷贝构造函数
    Frame::Frame(const Frame &other) {
        // 复制ID和时间戳等值类型
        id_ = other.id_;
        keyframe_id_ = other.keyframe_id_;
        time_stamp_ = other.time_stamp_;
        is_keyframe_ = other.is_keyframe_;

        // 复制位姿
        pose_ = other.pose_;
        // --- [修正] 使用正确的成员变量名 ---
        relative_pose_ = other.relative_pose_;

        // 深拷贝图像和描述子数据
        if (!other.left_img_.empty()) {
            left_img_ = other.left_img_.clone();
        }
        if (!other.right_img_.empty()) {
            right_img_ = other.right_img_.clone();
        }
        if (!other.descriptors_.empty()) {
            descriptors_ = other.descriptors_.clone();
        }

        // 深拷贝特征点列表
        for (const auto& feat : other.features_left_) {
            if (feat) {
                features_left_.push_back(std::make_shared<Feature>(*feat));
            }
        }
        for (const auto& feat : other.features_right_) {
            if (feat) {
                features_right_.push_back(std::make_shared<Feature>(*feat));
            }
        }
    }
}