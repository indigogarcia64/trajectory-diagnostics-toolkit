cmake_minimum_required(VERSION 3.17)

project(TrajectoryDiagnosticsToolkit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(trajectory_utils 
            TrajectoryUtils.cpp
            TrajectoryIO.cpp
            TrajectoryDiagnostics.cpp)

target_include_directories(trajectory_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(trajectory_diagnostics main.cpp)
target_link_libraries(trajectory_diagnostics PRIVATE trajectory_utils)

add_executable(trajectory_tests TrajectoryTests.cpp)
target_link_libraries(trajectory_tests PRIVATE trajectory_utils)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/data)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/data/trajectory_data_expansive.csv
    ${CMAKE_CURRENT_BINARY_DIR}/data/trajectory_data_expansive.csv
    COPYONLY
)
