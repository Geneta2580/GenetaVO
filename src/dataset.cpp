#include "myslam/dataset.h"

namespace myslam {

    Dataset::Dataset(
        const std::string& dataset_path)
        : dataset_path_(dataset_path) {}

    Frame::Ptr Dataset::loadNextFrame() {
        boost::format fmt("%s/image_%d/%06d.png"); // 规定路径格式
        cv::Mat image_left, image_right;

        image_left =
        cv::imread((fmt % dataset_path_ % 0 % current_index_).str(),
                   cv::IMREAD_GRAYSCALE);
        image_right =
            cv::imread((fmt % dataset_path_ % 1 % current_index_).str(),
                    cv::IMREAD_GRAYSCALE);

        if (image_left.data == nullptr || image_right.data == nullptr) {   // 判断指定路径图片是否存在
            std::cout << "cannot find images at index " << current_index_ << std::endl;
            return nullptr;
        }

        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left;
        new_frame->right_img_ = image_right;
        
        current_index_++; //图片读取完毕,id+1,这个id是私有变量不对外
        return new_frame;
    }
}