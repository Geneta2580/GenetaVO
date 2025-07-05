# add_library(myslam SHARED   # 创建myslam动态库
#         dataset.cpp
#         frame.cpp
#         feature.cpp
#         mappoint.cpp
#         config.cpp
#         frontend.cpp)

# include_directories(${PROJECT_SOURCE_DIR}/include)

# 创建动态库
add_library(myslam SHARED
    dataset.cpp
    frame.cpp
    feature.cpp
    mappoint.cpp
    config.cpp
    frontend.cpp
    camera.cpp
    map.cpp
    viewer.cpp
    backend.cpp
    loopclosing.cpp
)

<<<<<<< HEAD
# 链接到在根目录定义的第三方依赖集合
# PUBLIC 关键字会将 third_party_deps 的依赖属性（如 include 目录）传递给链接到 myslam 的任何目标（如 GenetaSLAM 可执行文件）
target_link_libraries(myslam 
    PUBLIC
    third_party_deps
=======
# 链接第三方库并传递依赖，比较重要，这里src目录下调用光流函数出问题，估计和库的配置有关
target_link_libraries(myslam 
    PUBLIC
    ${THIRD_PARTY_LIBS}
    DBoW3
>>>>>>> 723770ab0906c835a1ea9745461e7ef4041be37b
)