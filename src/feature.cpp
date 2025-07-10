#include "myslam/feature.h"

namespace myslam {

// 拷贝构造函数
Feature::Feature(const Feature &other) {
    frame_ = other.frame_; // weak_ptr可以直接复制
    position_ = other.position_; // KeyPoint是值类型，直接复制
    map_point_ = other.map_point_; // weak_ptr可以直接复制
    is_outlier_ = other.is_outlier_;
    is_on_left_image_ = other.is_on_left_image_;
}

} // namespace myslam