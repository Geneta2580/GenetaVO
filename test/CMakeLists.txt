# 启用测试框架（必须！）
enable_testing()

# 设置测试源文件列表
set(TEST_SOURCES mytest)  # 示例多个测试用例

foreach(test_src IN LISTS TEST_SOURCES)
    # 生成可执行文件
    add_executable(${test_src} ${test_src}.cpp)
    
    # 链接依赖库 - 使用现代化的目标链接方式
    target_link_libraries(${test_src} PRIVATE 
        myslam
        third_party_deps
    )
    
    # 注册测试用例（标准格式）
    add_test(
        NAME ${test_src}_Test          # 测试名称唯一标识
        COMMAND $<TARGET_FILE:${test_src}>  # 自动获取可执行文件路径
    )
endforeach()