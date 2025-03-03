cmake_minimum_required(VERSION 3.20)
project(Swarm)

set(CMAKE_CXX_STANDARD 17)



set(OpenCV_DIR "C:/opencv/x64/mingw/lib")
find_package(OpenCV REQUIRED)
set(OpenCV_INCLUDE_DIR C:/opencv/include)
include_directories(${OpenCV_INCLUDE_DIR})



add_executable(Swarm main.cpp Node.h SimMetrics.h SwarmMetrics.h BTevolution.h  BTevolutionTest.h blackboard.h Swarm.h BehaviorTree.h ModelTest.h rapidcsv.h csv.h csv2.h)


target_link_libraries(Swarm ${OpenCV_LIBS})


















