# 定义可执行文件
add_executable(GenetaSLAM run.cpp)

# 将可执行文件链接到 myslam 库和所有第三方依赖
# 这样写既干净又正确，可以解决所有 "undefined reference" 问题
target_link_libraries(GenetaSLAM
    PRIVATE
    myslam
    third_party_deps
)
