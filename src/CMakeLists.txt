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

# 链接到在根目录定义的第三方依赖集合
# PUBLIC 关键字会将 third_party_deps 的依赖属性（如 include 目录）传递给链接到 myslam 的任何目标（如 GenetaSLAM 可执行文件）
target_link_libraries(myslam 
    PUBLIC
    third_party_deps
)