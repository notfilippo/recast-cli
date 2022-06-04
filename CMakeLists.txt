cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

project(RecastCli VERSION 0.1.0)

set(RecastCli_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(RECASTNAVIGATION_DEMO OFF CACHE INTERNAL "")
set(RECASTNAVIGATION_TESTS OFF CACHE INTERNAL "")
set(RECASTNAVIGATION_EXAMPLES OFF CACHE INTERNAL "")

include(FetchContent)
FetchContent_Declare(recastnavigation
        GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation.git
        GIT_TAG        master
)
FetchContent_MakeAvailable(recastnavigation)
FetchContent_Declare(gflags
        GIT_REPOSITORY https://github.com/gflags/gflags.git
        GIT_TAG        master
        )
FetchContent_MakeAvailable(gflags)

add_executable(RecastCli
        include/build.h
        include/ChunkyTriMesh.h
        include/InputGeom.h
        include/MeshLoaderObj.h
        src/build.cpp
        src/ChunkyTriMesh.cpp
        src/InputGeom.cpp
        src/main.cpp
        src/MeshLoaderObj.cpp
)

target_link_libraries(RecastCli Recast Detour gflags::gflags)
target_include_directories(RecastCli PRIVATE ${RecastCli_INCLUDE_DIR} ${Recast_INCLUDE_DIR} ${Detour_INCLUDE_DIR})
