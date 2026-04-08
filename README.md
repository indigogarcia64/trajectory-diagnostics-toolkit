#Trajectory Diagnostics Toolkit

This project is a diagnostics toolkit primarily for analyzing anomalous behavior in trajectory vectors.
It expects a CSV file with four categories, formatted in timestamp, x, y, theta. Each row represents a particular pose at a particular time.
The output provides a lists of anomalous behaviors including total anomalies and anomaly type. 
This toolkit was developed as a diagnostics expansion of a project that only provided basic utilities such as finding the minimum x coordinate
or finding the maximum x coordinate. 

Assumptions:
1. Data file MUST be in CSV in format:

timestamp,x,y,theta

where...

timestamp: the time at which the pose measurement was taken (in seconds)
x: the x coordinate
y: the y coordinate
theta: the heading angle

You may define your own units for the spatial coordinates, as this toolkit is not unit dependent except for on time.

2. Data must be of type double. There is currently no support or case-handling for non-double types such as int, std::string, or custom types.


Tests are custom/manual and do not use frameworks such as GoogleTest.


Features:
-Jump anomaly detection with adjustable threshold

-Time anomaly detection with adjustable threshold

-Total diagnostic summary reporting and print of anomalies

-Custom CSV data parser

-Trajectory printing

-Sort by x pose

-Shifting the trajectory's x coordinates

-Computing the average x and y coordinate

-Finding the min/max x coordinate

-Empty-trajectory handling with std::optional

-Unit tests


##File structure

'TimedPose.hpp':
Unit pose struct definition.

'TrajectoryDiagnostics.hpp':
Function definitions for diagnostics.

'TrajectoryDiagnostics.cpp':
Function implementations for diagnostics.

'TrajectoryIO.hpp':
Function definition for parser.

'TrajectoryIO.cpp':
Function implementation for parser.

'TrajectoryTests.cpp':
Unit tests for all diagnostic functions including empty trajectory, nullopt, and single element cases. 

'TrajectoryUtils.hpp':
Function definitions for trajectory utilities such as maximum x, minimum x, shift x, etc.

'TrajectoryUtils.cpp':
Function implementations for trajectory utilities.

'main.cpp':
Demo executable.

'CMakeLists.txt':
Build configuration.

##Build instructions
mkdir data {copy csv file into THIS directory, not the source!}
mkdir build
cd build
cmake .. 
cmake --build .

##Run instructions
./trajectory_diagnostics ../{CSV FILE NAME}
./trajectory_tests