# 测试可执行文件
add_executable(GenetaSLAM run.cpp) 

target_link_libraries(GenetaSLAM myslam ${THIRD_PARTY_LIBS})