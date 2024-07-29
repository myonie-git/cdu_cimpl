cmake_minimum_required(VERSION 3.10)

# 项目信息
project(CDU_CIMPL)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE "Debug")

# 手动指定TinyXML2的位置
set(TINYXML2_INCLUDE_DIR /usr/include/tinyxml2)
set(TINYXML2_LIBRARIES /usr/lib/x86_64-linux-gnu/libtinyxml2.so)

include_directories(${TINYXML2_INCLUDE_DIR})

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
if(WIN32)
    # Correct directory separator for Windows
    string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
endif()
configure_file(resources/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/resources/config.h")
include_directories(${CMAKE_CURRENT_BINARY_DIR})



# Set compile options
set(PROJECT_COMPILE_OPTIONS
  -Wall
  -Wextra
  -Wwrite-strings
  -Wunreachable-code
  -Wpointer-arith
  -Wredundant-decls
  -Wno-unused-parameter
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/geometric_shapes/cmake/")

# used for finding Assimp and FCL
find_package(PkgConfig REQUIRED)

# TODO: Remove policy fix when assimp 5.1 is available
set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
endif()

include_directories(${PROJECT_SOURCE_DIR})


find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

find_package(Boost REQUIRED system filesystem)

find_package(console_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(octomap REQUIRED)

pkg_check_modules(LIBFCL_PC REQUIRED fcl)
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

find_package(QHULL REQUIRED)
find_package(random_numbers REQUIRED)
find_package(resource_retriever REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS} ${LIBFCL_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS}
  ${random_numbers_INCLUDE_DIRS} ${resource_retriever_INCLUDE_DIRS})

# 查找Eigen3库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 包含Eigen头文件目录
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加geometric_shapes目录
add_subdirectory(geometric_shapes)

# 包含geometric_shapes头文件目录
include_directories(${PROJECT_SOURCE_DIR}/geometric_shapes/include)

# 包含geometric_shapes头文件目录
include_directories(${PROJECT_SOURCE_DIR}/geometric_shapes/test)

# 设置调试信息
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 源文件
set(SOURCES
    src/aabb.cpp
    src/obb.cpp
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

# 链接Eigen库、tinyxml2库和geometric_shapes库
target_link_libraries(cdu_cimpl Eigen3::Eigen ${TINYXML2_LIBRARIES} geometric_shapes)

# 定义一个函数来简化添加测试可执行文件的过程
function(add_test_executable test_name)
    set(TEST_SOURCES
        test/${test_name}.cpp
        src/aabb.cpp
        src/obb.cpp
        src/bvh_node_base.cpp
        src/bvh_tree.cpp
        src/collision_env.cpp
        src/collision_object.cpp
        src/urdf_model.cpp
    )
    add_executable(${test_name} ${TEST_SOURCES})
    target_link_libraries(${test_name} Eigen3::Eigen ${TINYXML2_LIBRARIES} geometric_shapes)
endfunction()

# 添加测试可执行文件
add_test_executable(test_build_env)
add_test_executable(test_random_env)
add_test_executable(test_urdf_model)
add_test_executable(test_meshes_to_obb)
add_test_executable(test_collision_check)
