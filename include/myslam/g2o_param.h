#ifndef MYSLAM_G2O_PARAM_H
#define MYSLAM_G2O_PARAM_H

#include "myslam/common.h"

namespace myslam {
    // vertex and edges used in g2o ba 重定义顶点及其更新运算
    class VertexPose : public g2o::BaseVertex<6, SE3> { // 注意位姿的李代数是6维度，3维的SO3d加上三维的平移向量
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            
            virtual void setToOriginImpl() override {
                _estimate = SE3();  // 显式设置合理初始值
            }
            
            /// left multiplication on SE3
            virtual void oplusImpl(const double *update) override {
                Vec6 delta;
                delta << update[0], update[1], update[2], update[3], update[4], update[5]; // 李代数更新
                _estimate = SE3::exp(delta) * _estimate;  // 这里是李代数小扰动左乘更新
            }
            
            virtual bool read(std::istream &in) override {return true;}
            
            virtual bool write(std::ostream &out) const override {return true;}
    };

    // 重定义仅估计位姿的一元边
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose> {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            EdgeProjectionPoseOnly(const Vec3& p_world, Mat33& K) : _p_world(p_world),  _K(K){} // 定义外部传入的变量

            virtual void computeError() override {
                const VertexPose *v = static_cast<VertexPose *> ( _vertices[0] );
                SE3 T = v->estimate(); // 优化顶点的估计值，这里是指位姿变换的估计
                Vec3 p_homo = _K * (T * _p_world); // 投影变换后的齐次坐标
                Vec2 p_image( // 转换为非齐次坐标
                    p_homo.x() / p_homo.z(),
                    p_homo.y() / p_homo.z()
                );
                _error = _measurement - p_image; // 估计值和目标值之差
            }
            
            virtual void linearizeOplus() override {  // 仅估计位姿边的雅可比，提升计算效率，非常重要
                const VertexPose *v = static_cast<VertexPose *>(_vertices[0]);
                SE3 T = v->estimate();
                Vec3 pos_cam = T * _p_world;
                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pos_cam[0];
                double Y = pos_cam[1];
                double Z = pos_cam[2];
                double Zinv = 1.0 / (Z + 1e-18);
                double Zinv2 = Zinv * Zinv;
                // 按行填充，每行6个元素
                _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
            }

            bool read(std::istream &in) override {return true;}
            
            bool write(std::ostream &out) const override{return true;}
    
        private:
            Vec3 _p_world; // 注意外部传入变量要在private当中声明
            Mat33 _K;
    };

    /// 后端优化用
    /// 路标顶点
    class VertexXYZ : public g2o::BaseVertex<3, Vec3> {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            virtual void setToOriginImpl() override { _estimate = Vec3::Zero(); } // 设置路标顶点初值

            virtual void oplusImpl(const double *update) override { // 路标顶点更新规则，增量式更新
                _estimate[0] += update[0];
                _estimate[1] += update[1];
                _estimate[2] += update[2];
            }

            virtual bool read(std::istream &in) override { return true; }

            virtual bool write(std::ostream &out) const override { return true; }
    };

    /// 重定义位姿边，估计地图和位姿的二元边
    class EdgeProjection: public g2o::BaseBinaryEdge<2, Vec2, VertexPose, VertexXYZ> {

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            /// 构造时传入相机内外参
            EdgeProjection(const Mat33& K, const SE3& cam_ext) : _K(K), _cam_ext(cam_ext) {} // 定义外部传入的变量
                                                                                        // 注意由于这里左右目都要优化，所以要传相机位姿
            // 定义重投影误差
            virtual void computeError() override { 
                const VertexPose *v0 = static_cast<VertexPose *> ( _vertices[0] );
                const VertexXYZ *v1 = static_cast<VertexXYZ *> ( _vertices[1] );
                SE3 T = v0->estimate();
                Vec3 p_homo = _K * (_cam_ext * (T * v1->estimate())); // 投影变换后的齐次坐标
                Vec2 p_image( // 转换为非齐次坐标
                    p_homo.x() / p_homo.z(),
                    p_homo.y() / p_homo.z()
                );
                _error = _measurement - p_image; // 估计值和目标值之差
            }

            // 手写雅可比
            virtual void linearizeOplus() override {
                const VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
                const VertexXYZ *v1 = static_cast<VertexXYZ *>(_vertices[1]);
                SE3 T = v0->estimate();
                Vec3 pw = v1->estimate();
                Vec3 pos_cam = _cam_ext * T * pw;
                double fx = _K(0, 0);
                double fy = _K(1, 1);
                double X = pos_cam[0];
                double Y = pos_cam[1];
                double Z = pos_cam[2];
                double Zinv = 1.0 / (Z + 1e-18);
                double Zinv2 = Zinv * Zinv;
                _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                    -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                    fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                    -fy * X * Zinv;
        
                _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                                _cam_ext.rotationMatrix() * T.rotationMatrix();
            }

            virtual bool read(std::istream &in) override { return true; }

            virtual bool write(std::ostream &out) const override { return true; }

        private:
            Mat33 _K;
            SE3 _cam_ext;
    };

    // 回环边
    class LoopEdge : public g2o::BaseBinaryEdge<6, SE3, VertexPose, VertexPose> {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            // 误差计算（观测值-估计值）
            virtual void computeError() override {
                const VertexPose* v1 = static_cast<const VertexPose*>(_vertices[0]);
                const VertexPose* v2 = static_cast<const VertexPose*>(_vertices[1]);
                
                // 获取顶点位姿
                SE3 T1 = v1->estimate();
                SE3 T2 = v2->estimate();
                
                // 计算相对位姿误差 _measurement.inverse() * 
                _error = (T1.inverse() * T2).log();
            }
            
            virtual bool read(std::istream &in) override { return true; }

            virtual bool write(std::ostream &out) const override { return true; }
    };

}

#endif