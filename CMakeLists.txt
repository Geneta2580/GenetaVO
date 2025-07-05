cmake_minimum_required(VERSION 3.12)
project(myslam)

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBow3)

set(CMAKE_BUILD_TYPE release) # release "Debug"
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# --- 查找所有依赖库 ---
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(g2o REQUIRED COMPONENTS core solver_csparse types_sba csparse_extension)

# --- 终极修复：手动指定 Glog 路径，绕过 find_package 的问题 ---
# find_package(Glog REQUIRED) # 注释掉有问题的 find_package
set(GLOG_INCLUDE_DIRS "/usr/include")
set(GLOG_LIBRARIES "/usr/lib/x86_64-linux-gnu/libglog.so")
# ----------------------------------------------------------------

# --- 诊断信息：打印 CMake 找到的 Glog 库路径 ---
message(STATUS "Glog libraries found: ${GLOG_LIBRARIES}")
message(STATUS "Glog include dirs found: ${GLOG_INCLUDE_DIRS}")
# ----------------------------------------------------

find_package(fmt REQUIRED) # 假设你也需要 fmt 库

# --- 删除旧的 include_directories(...) 和 set(THIRD_PARTY_LIBS ...) ---

# --- 创建一个 INTERFACE 库来管理所有第三方依赖 ---
add_library(third_party_deps INTERFACE)

# 将第三方库的头文件目录添加为 SYSTEM，以抑制其编译警告
target_include_directories(third_party_deps SYSTEM INTERFACE
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/src
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    "/usr/include/eigen3/"
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
)

# 将所有需要链接的第三方库添加到 INTERFACE 库
target_link_libraries(third_party_deps INTERFACE
    DBoW3
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
    pthread
    ${CSPARSE_LIBRARY}
    ${GLOG_LIBRARIES} # Glog 库在这里被添加
    fmt::fmt
)

# --- 处理你自己的项目 ---
include_directories(${PROJECT_SOURCE_DIR}/include) # 只保留对自己项目的 include

add_subdirectory(src)
add_subdirectory(app)
add_subdirectory(test)
