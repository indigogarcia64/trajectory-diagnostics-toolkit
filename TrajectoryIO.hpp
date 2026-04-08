#pragma once

#include <optional>
#include <vector>
#include <string>

#include "TimedPose.hpp"

std::optional<std::vector<TimedPose>> loadTrajectoryFromCSV(const std::string& filename);