#pragma once

#include <vector>
#include <string>
#include <optional>

#include "TimedPose.hpp"

void printTrajectory(const std::vector<TimedPose>& trajectory);
void printOptionalMetric(const std::string& label, const std::optional<double>& value);
void shiftX(std::vector<TimedPose>& trajectory, double dx);
bool compareByX(const TimedPose& a, const TimedPose& b);
std::optional<double> computeAverageX(const std::vector<TimedPose>& trajectory);
std::optional<double> computeAverageY(const std::vector<TimedPose>& trajectory);
std::optional<double> findMinX(const std::vector<TimedPose>& trajectory);
std::optional<double> findMaxX(const std::vector<TimedPose>& trajectory);
std::optional<double> computeTotalPathLength(const std::vector<TimedPose>& trajectory);

