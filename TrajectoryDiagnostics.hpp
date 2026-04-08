#pragma once

#include "TrajectoryUtils.hpp"

constexpr double JUMP_THRESHOLD = 2.0;      //meters
constexpr double TIME_JUMP_THRESHOLD = 0.5; //seconds

struct JumpAnomaly{
    double prev_timestamp;
    double curr_timestamp;
    double distance;
};

enum class TimestampAnomalyType{
    Repeated,
    Backward,
    Oversized
};

struct TimestampAnomaly{
    double prev_timestamp;
    double curr_timestamp;
    double time_gap;
    TimestampAnomalyType type;
};

struct DiagnosticSummary{
    int totalAnomalyCount = 0;
    int jumpCount = 0;
    int repeatedTimeCount = 0;
    int backwardTimeCount = 0;
    int oversizedTimeCount = 0;
};

std::optional<std::vector<JumpAnomaly>> detectJumpAnomaly(const std::vector<TimedPose>& trajectory);
std::optional<std::vector<TimestampAnomaly>> detectTimeAnomaly(const std::vector<TimedPose>& trajectory);
void updateDiagnosticSummary(DiagnosticSummary& summary, const std::optional<std::vector<JumpAnomaly>>& jumpAnomalies);
void updateDiagnosticSummary(DiagnosticSummary& summary, const std::optional<std::vector<TimestampAnomaly>>& timeAnomalies);
void printAnomalies(const std::vector<JumpAnomaly>& anomalies);
void printAnomalies(const std::vector<TimestampAnomaly>& anomalies);
void printSummary(const DiagnosticSummary& summary);
void reportAnomalies(const std::optional<std::vector<JumpAnomaly>>& anomalies);
void reportAnomalies(const std::optional<std::vector<TimestampAnomaly>>& anomalies);