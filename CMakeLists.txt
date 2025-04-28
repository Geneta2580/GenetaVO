cmake_minimum_required(VERSION 3.12)
project(myslam)

add_subdirectory(${PROJECT_SOURCE_DIR}/Thirdparty/DBow3)

set(CMAKE_BUILD_TYPE Release)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin) # 设置可执行文件输出目录(make后的文件)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib) # 将myslam封装成库,输出在lib目录下

# 查找依赖库
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(g2o REQUIRED COMPONENTS 
    core 
    solver_csparse 
    types_sba
    csparse_extension
)
find_package(Glog REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBow3/src 
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    "/usr/include/eigen3/"
    ${Pangolin_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS     # 把以下库封装成第三方库
        DBoW3
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff 
        g2o_types_sba 
        g2o_solver_csparse 
        g2o_csparse_extension
        pthread
        ${CSPARSE_LIBRARY}
        ${GLOG_INCLUDE_DIRS}
        )

enable_testing()


############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${DBoW2_INCLUDE_DIRS})

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)  # app文件夹下的程序也会在/bin目录下生成一个可执行文件