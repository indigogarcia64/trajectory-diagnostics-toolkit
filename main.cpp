#include <iostream>

#include "TrajectoryIO.hpp"
#include "TrajectoryUtils.hpp"
#include "TrajectoryDiagnostics.hpp"

int main(int argc, char* argv[]){
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <trajectory_file.csv>" << std::endl;
        return 1;
    }

    std::string file = argv[1];
    auto trajectory = loadTrajectoryFromCSV(file);

    if (!trajectory) {
        std::cout << "Failed to load " << file << ".\n" << std::endl;
        return 1;
    }

    printTrajectory(*trajectory);

    printOptionalMetric("Average x coordinate", computeAverageX(*trajectory));
    printOptionalMetric("Average y coordinate", computeAverageY(*trajectory));
    printOptionalMetric("Min x", findMinX(*trajectory));
    printOptionalMetric("Max x", findMaxX(*trajectory));
    printOptionalMetric("Total path length", computeTotalPathLength(*trajectory));

    auto jumpAnomalies = detectJumpAnomaly(*trajectory);
    auto timeAnomalies = detectTimeAnomaly(*trajectory);

    reportAnomalies(jumpAnomalies);
    reportAnomalies(timeAnomalies);

    DiagnosticSummary summary;

    updateDiagnosticSummary(summary, jumpAnomalies);
    updateDiagnosticSummary(summary, timeAnomalies);

    printSummary(summary);

    return 0;
}