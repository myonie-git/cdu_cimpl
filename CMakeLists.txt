cmake_minimum_required(VERSION 3.10)

# 项目信息
project(CDU_CIMPL)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含目录
include_directories(include)

# 查找Eigen3库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 包含Eigen头文件目录
include_directories(${EIGEN3_INCLUDE_DIR})

# 设置调试信息
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 源文件
set(SOURCES
    src/aabb.cpp
    src/bvh_node_base.cpp
    src/bvh_tree.cpp
    src/collision_env.cpp
    src/collision_object.cpp
    src/main.cpp
)

# 生成主程序可执行文件
add_executable(cdu_cimpl ${SOURCES})

# 链接Eigen库
target_link_libraries(cdu_cimpl Eigen3::Eigen)

# 测试文件源文件
set(TEST_SOURCES
    test/test_build_env.cpp
    src/aabb.cpp
    src/bvh_node_base.cpp
    src/bvh_tree.cpp
    src/collision_env.cpp
    src/collision_object.cpp
)

# 添加测试可执行文件
add_executable(test_build_env ${TEST_SOURCES})
target_link_libraries(test_build_env Eigen3::Eigen)
