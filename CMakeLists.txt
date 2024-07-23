cmake_minimum_required(VERSION 3.10)
project(remove_dynamic_points)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 添加 json.hpp 头文件所在的目录到包含路径
include_directories(${PROJECT_SOURCE_DIR}/include)

# 查找 PCL 库
find_package(PCL 1.8 REQUIRED)

# 包含 PCL 的头文件和链接目录
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加源文件
add_executable(remove_dynamic_points src/remove_dynamic_points.cpp)

# 链接 PCL 库
target_link_libraries(remove_dynamic_points ${PCL_LIBRARIES})
