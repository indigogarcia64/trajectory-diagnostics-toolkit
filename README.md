# Trajectory Diagnostics Toolkit

## Overview
A C++ diagnostics tool for analyzing time-stamped 2D trajectory data. The toolkit loads trajectory logs from CSV, computes basic motion metrics, detects geometric and temporal anomalies, and prints a summary report. It was developed as an extension of an earlier trajectory-utilities project into a more complete diagnostics and validation tool.

## Features

- CSV trajectory loading
- Trajectory printing
- Average x and y coordinate computation
- Minimum and maximum x coordinate detection
- Total path length computation
- Jump anomaly detection
- Timestamp anomaly detection:
  - repeated timestamps
  - backward timestamps
  - oversized timestamp gaps
- Diagnostic summary reporting
- Command-line file input
- Unit-style tests for metrics, anomaly detection, and summary logic

## CSV Format

The toolkit expects a CSV file in the following format:

```text
timestamp,x,y,theta
0.0,0.0,0.0,0.0
0.1,0.1,0.0,0.0
0.2,0.2,0.0,0.0
```

## Build instructions
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Run instructions
Analyze a trajectory file:
```bash
./trajectory_diagnostics ../data/trajectory_data.csv
```

Run the test executable:
```bash
./trajectory_tests
```

### Example Output
```Average x coordinate: 0.25
Average y coordinate: 0.125
Total path length: 0.34

Jump anomalies detected!
Anomaly type: Jump
Occurred between: 0.2s and 0.3s
Jump distance: 3.2m

DIAGNOSTIC SUMMARY
---------------------------------------------------------
Total anomaly count: 3
Jump count: 1
Repeated time count: 1
Backward time count: 0
Oversized time count: 1
```

## Diagnostic Logic
The toolkit currently detects:
- **Jump Anomaly**: consecutive spatial displacement exceeds `JUMP_THRESHOLD`
- **Repeated Timestamp**: consecutive timestamps are identical
- **Backward Timestamp**: current timestamp is less than previous timestamp
- **Oversized Timestamp**: consecutive timestamp difference exceeds `TIME_JUMP_THRESHOLD` 

## File structure
- 'TimedPose.hpp': Timed-stamped pose data structure.
- 'TrajectoryDiagnostics.hpp/cpp': Anomaly detection, reporting, and summary logic.
- 'TrajectoryIO.hpp/cpp': CSV parsing and trajectory loading.
- 'TrajectoryTests.cpp': Unit style tests.
- 'TrajectoryUtils.hpp/cpp': Metric and utility functions.
- 'main.cpp': Demo executable.
- 'CMakeLists.txt': Build configuration.

## Testing
Tests currently cover:
- total path length
- jump anomaly detection
- time anomaly detection
- diagnostic summary updates

## Limitations/Future Work:
- Fixed CSV schema: timestamp,x,y,theta
- Malformed rows cause loading failure
- Thresholds are compile-time constraints
- No plotting or visualization yet
- No velocity, acceleration, or heading-change diagnostics yet
- No general-purpose CSV quoting or dialect support


