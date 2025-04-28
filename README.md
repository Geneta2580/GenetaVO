# GenetaVO
参考高博视觉SLAM十四讲ch13代码结构手写的VO，效果比较一般，使用ORB特征子进行计算，LK光流进行特征点追踪，调用OpenCV的三角化接口，使用g2o库进行前、后端计算优化，同时包含前端、后端、viewer、loopclosing（可选且未开发完全）四个线程


在实现速度上有一定优化，将地图更新这类比较吃时间的操作从前端移到了后端和viewer线程，加速前端线程的运行

## 环境主要配置参考
Ubuntu24.04

OpenCV 4.10.0

Ceres

g2o

Sophus

以上均需安装在系统目录下

![运行演示图](https://github.com/user-attachments/assets/f9ec1e13-c758-4a58-994e-81a41d6d8233)
