cmake_minimum_required(VERSION 3.10)

# 项目信息
project(CDU_CIMPL)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE "Debug")

# 手动指定TinyXML2的位置
set(TINYXML2_INCLUDE_DIR /usr/include/tinyxml2)
set(TINYXML2_LIBRARIES /usr/lib/x86_64-linux-gnu/libtinyxml2.so)

include_directories(${TINYXML2_INCLUDE_DIR})

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
    src/urdf_model.cpp
    src/main.cpp
)

set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

# 生成主程序可执行文件
add_executable(cdu_cimpl ${SOURCES})

# 链接Eigen库和tinyxml2库
target_link_libraries(cdu_cimpl Eigen3::Eigen ${TINYXML2_LIBRARIES})

# 定义一个函数来简化添加测试可执行文件的过程
function(add_test_executable test_name)
    set(TEST_SOURCES
        test/${test_name}.cpp
        src/aabb.cpp
        src/bvh_node_base.cpp
        src/bvh_tree.cpp
        src/collision_env.cpp
        src/collision_object.cpp
        src/urdf_model.cpp
    )
    add_executable(${test_name} ${TEST_SOURCES})
    target_link_libraries(${test_name} Eigen3::Eigen ${TINYXML2_LIBRARIES})
endfunction()

# 添加测试可执行文件
add_test_executable(test_build_env)
add_test_executable(test_random_env)
add_test_executable(test_urdf_model)
