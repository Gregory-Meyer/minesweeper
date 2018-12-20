cmake_minimum_required(VERSION 3.8)

project(minesweeper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

include_directories(include/ external/GSL/include/ ${Qt5_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIRS})

add_executable(minesweeper src/main.cpp src/util/state.cpp)
target_link_libraries(minesweeper ${Qt5_LIBRARIES} ${Eigen3_LIBRARIES})
