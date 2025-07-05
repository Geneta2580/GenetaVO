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
}