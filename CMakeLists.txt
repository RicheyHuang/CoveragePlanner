cmake_minimum_required(VERSION 3.5)
project(CoveragePlanner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CGAL QUIET COMPONENTS Core)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CGAL_INCLUDE_DIRS})

file(GLOB_RECURSE srcs "src/*.cc" "src/*.cpp")
file(GLOB_RECURSE hdrs "include/*.h")

message("find source files: ${srcs}")
message("find headers: ${hdrs}")

include_directories(include)

add_executable(CoveragePlanner ${srcs} ${hdrs})

target_link_libraries(CoveragePlanner
        ${OpenCV_LIBS}
        CGAL::CGAL
        CGAL::CGAL_Core
        )

