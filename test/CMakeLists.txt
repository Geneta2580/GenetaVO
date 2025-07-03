# 启用测试框架（必须！）
enable_testing()

# 设置测试源文件列表
set(TEST_SOURCES mytest)  # 示例多个测试用例

foreach(test_src IN LISTS TEST_SOURCES)
    # 生成可执行文件
    add_executable(${test_src} ${test_src}.cpp)
    
    # 链接依赖库
    target_link_libraries(${test_src} PRIVATE 
        ${THIRD_PARTY_LIBS} 
        myslam  # 确保myslam已通过add_subdirectory定义
    )
    
    # 注册测试用例（标准格式）
    add_test(
        NAME ${test_src}_Test          # 测试名称唯一标识
        COMMAND $<TARGET_FILE:${test_src}>  # 自动获取可执行文件路径
    )
endforeach()