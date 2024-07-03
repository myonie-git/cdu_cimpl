cmake_minimum_required(VERSION 3.10)

# 项目信息
project(CDU_CIMPL)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含目录
include_directories(include)

# 源文件
set(SOURCES
    src/transform.cpp
    src/main.cpp
)

# 生成可执行文件
add_executable(cdu_cimpl ${SOURCES})

# 如果需要添加库链接，取消以下注释并调整库名称
# target_link_libraries(cdu_cimpl <library_name>)
