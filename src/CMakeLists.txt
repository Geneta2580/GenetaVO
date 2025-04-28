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

# 链接第三方库并传递依赖，比较重要，这里src目录下调用光流函数出问题，估计和库的配置有关
target_link_libraries(myslam 
    PUBLIC
    ${THIRD_PARTY_LIBS}
    DBoW3
)