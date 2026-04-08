#include <iostream>
#include <vector>
#include <optional>
#include <cmath>
#include "TrajectoryIO.hpp"
#include "TrajectoryUtils.hpp"
#include "TrajectoryDiagnostics.hpp"

bool nearlyEqual(double a, double b, double epsilon = 1e-4) {
    return std::abs(a - b) < epsilon;
}

void testComputeTotalPathLength(){
    auto trajectory = loadTrajectoryFromCSV("data/trajectory_data.csv");

    if (nearlyEqual(*computeTotalPathLength(*trajectory), 0.341421)){
        std::cout << "ComputeTotalPathLength test: passed" << std::endl;
    }
    else{
        std::cout << "ComputeTotalPathLength test: failed" << std::endl;
    }
}

void testComputeTotalPathLength_SingleElement(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {4.0, 3.0, 0.5}});

    if (nearlyEqual(*computeTotalPathLength(trajectory), 0.0)){
        std::cout << "ComputeTotalPathLength_SingleElement test: passed" << std::endl;
    }
    else{
        std::cout << "ComputeTotalPathLength_SingleElement test: failed" << std::endl;
    }
}

void testComputeTotalPathLength_Empty(){
    std::vector<TimedPose> empty;

    if (computeTotalPathLength(empty)){
        std::cout << "ComputeTotalPathLength_Empty test: failed" << std::endl;
    }
    else{
        std::cout << "ComputeTotalPathLength_Empty test: passed" << std::endl;
    }
}

void testDetectJumpAnomaly_NoJumps(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({1.0, {0.2, 0.15, 0.1}});
    trajectory.push_back({2.0, {0.3, 0.2, 0.15}});

    std::optional<std::vector<JumpAnomaly>> anomaly = detectJumpAnomaly(trajectory);

    if (anomaly->empty()){
        std::cout << "DetectJumpAnomaly_NoJumps test: passed" << std::endl;
    }
    else{
        std::cout << "DetectJumpAnomaly_NoJumps test: failed" << std::endl;
    }
}

void testDetectJumpAnomaly_SingleJump(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({1.0, {0.2, 5.0, 0.1}});
    trajectory.push_back({2.0, {0.3, 5.5, 0.15}});

    std::optional<std::vector<JumpAnomaly>> anomaly = detectJumpAnomaly(trajectory);

    if(anomaly){
        if (anomaly->size() == 1){
            std::cout << "DetectJumpAnomaly_SingleJump test: passed" << std::endl;
        }
        else{
            std::cout << "DetectJumpAnomaly_SingleJump test: failed" << std::endl;
        }
    }
    else{
        std::cout << "\nThere are no anomalies.\n" << std::endl;
    }
}

void testDetectJumpAnomaly_Empty(){
    std::vector<TimedPose> empty;

    std::optional<std::vector<JumpAnomaly>> anomaly = detectJumpAnomaly(empty);

    if (!anomaly){
        std::cout << "DetectJumpAnomaly_Empty test: passed" << std::endl;
    }
    else{
        std::cout << "DetectJumpAnomaly_Empty test: failed" << std::endl;
    }
}

void testDetectJumpAnomaly_OnePose(){
    std::vector<TimedPose> pose;

    pose.push_back({0.0, {5.0, 2.0, 3.0}});

    std::optional<std::vector<JumpAnomaly>> anomaly = detectJumpAnomaly(pose);

    if (anomaly->empty()){
        std::cout << "DetectJumpAnomaly_OnePose test: passed" << std::endl;
    }
    else{
        std::cout << "DetectJumpAnomaly_OnePose test: failed" << std::endl;
    }
}

void testDetectJumpAnomaly_Boundary(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.0, 0.0, 0.05}});

    trajectory.push_back({1.0, {1.41421356237,
                                1.41421356237,
                                0.1}});

    trajectory.push_back({2.0, {0.3, 1.0, 0.15}});

    std::optional<std::vector<JumpAnomaly>> anomaly = detectJumpAnomaly(trajectory);

    if (anomaly->empty()){
        std::cout << "DetectJumpAnomaly_Boundary test: passed" << std::endl;
    }
    else{
        std::cout << "DetectJumpAnomaly_Boundary test: failed" << std::endl;
    }
}

void testDetectTimeAnomaly_NoAnomalies(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({0.1, {0.2, 0.15, 0.1}});
    trajectory.push_back({0.2, {0.3, 0.2, 0.15}});
    trajectory.push_back({0.3, {0.4, 0.25, 0.2}});

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(trajectory);

    if (anomaly->empty()){
        std::cout << "DetectTimeAnomaly_NoAnomalies test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_NoAnomalies test: failed" << std::endl;
    }
}
void testDetectTimeAnomaly_Empty(){
    std::vector<TimedPose> empty;

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(empty);

    if (anomaly == std::nullopt){
        std::cout << "DetectTimeAnomaly_Empty test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_Empty test: failed" << std::endl;
    }
}

void testDetectTimeAnomaly_SingleElement(){
    std::vector<TimedPose> pose;

    pose.push_back({0.1, {0.1, 0.4, 0.2}});

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(pose);

    if (anomaly->empty()){
        std::cout << "DetectTimeAnomaly_SingleElement test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_SingleElement test: failed" << std::endl;
    }
}

void testDetectTimeAnomaly_Repeated(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({0.1, {0.2, 0.15, 0.1}});
    trajectory.push_back({0.1, {0.3, 0.2, 0.15}});
    trajectory.push_back({0.2, {0.4, 0.25, 0.2}});

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(trajectory);

    if ((*anomaly)[0].type == TimestampAnomalyType::Repeated){
        std::cout << "DetectTimeAnomaly_Repeated test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_Repeated test: failed" << std::endl;
    }
}

void testDetectTimeAnomaly_Backward(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({0.1, {0.2, 0.15, 0.1}});
    trajectory.push_back({0.2, {0.3, 0.2, 0.15}});
    trajectory.push_back({0.1, {0.4, 0.25, 0.2}});

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(trajectory);

    if ((*anomaly)[0].type == TimestampAnomalyType::Backward){
        std::cout << "DetectTimeAnomaly_Backward test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_Backward test: failed" << std::endl;
    }
}

void testDetectTimeAnomaly_Oversized(){
    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({0.1, {0.2, 0.15, 0.1}});
    trajectory.push_back({10.0, {0.3, 0.2, 0.15}});
    trajectory.push_back({10.1, {0.4, 0.25, 0.2}});

    std::optional<std::vector<TimestampAnomaly>> anomaly = detectTimeAnomaly(trajectory);

    if ((*anomaly)[0].type == TimestampAnomalyType::Oversized){
        std::cout << "DetectTimeAnomaly_Oversized test: passed" << std::endl;
    }
    else{
        std::cout << "DetectTimeAnomaly_Oversized test: failed" << std::endl;
    }
}

void testDiagnosticSummary_NoAnomalies(){
    DiagnosticSummary summary;

    std::vector<JumpAnomaly> jump_anomalies;
    std::vector<TimestampAnomaly> time_anomalies;

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 0){
        std::cout << "DiagnosticSummary_NoAnomalies test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_NoAnomalies test: failed" << std::endl;
    }
}

void testDiagnosticSummary_OneJumpAnomaly(){
    DiagnosticSummary summary;

    std::vector<JumpAnomaly> jump_anomalies;
    std::vector<TimestampAnomaly> time_anomalies;

    JumpAnomaly anomaly{0.0, 1.0, 3.0};
    jump_anomalies.push_back(anomaly);

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 1 && summary.jumpCount == 1){
        std::cout << "DiagnosticSummary_OneJumpAnomaly test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_OneJumpAnomaly test: failed" << std::endl;
    }
}

void testDiagnosticSummary_RepeatedTimeAnomaly(){
    DiagnosticSummary summary;

    std::vector<JumpAnomaly> jump_anomalies;
    std::vector<TimestampAnomaly> time_anomalies;

    TimestampAnomaly anomaly{0.0, 0.0, 0.0, TimestampAnomalyType::Repeated};
    time_anomalies.push_back(anomaly);

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 1 && summary.repeatedTimeCount == 1){
        std::cout << "DiagnosticSummary_RepeatedTimeAnomaly test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_RepeatedTimeAnomaly test: failed" << std::endl;
    }
}

void testDiagnosticSummary_BackwardTimeAnomaly(){
    DiagnosticSummary summary;

    std::vector<JumpAnomaly> jump_anomalies;
    std::vector<TimestampAnomaly> time_anomalies;

    TimestampAnomaly anomaly{0.0, -1.0, -1.0, TimestampAnomalyType::Backward};
    time_anomalies.push_back(anomaly);

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 1 && summary.backwardTimeCount == 1){
        std::cout << "DiagnosticSummary_BackwardTimeAnomaly test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_BackwardTimeAnomaly test: failed" << std::endl;
    }
}

void testDiagnosticSummary_OversizedTimeAnomaly(){
    DiagnosticSummary summary;

    std::vector<JumpAnomaly> jump_anomalies;
    std::vector<TimestampAnomaly> time_anomalies;

    TimestampAnomaly anomaly{0.0, 10.0, 10.0, TimestampAnomalyType::Oversized};
    time_anomalies.push_back(anomaly);

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 1 && summary.oversizedTimeCount == 1){
        std::cout << "DiagnosticSummary_OversizedTimeAnomaly test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_OversizedTimeAnomaly test: failed" << std::endl;
    }
}

void testDiagnosticSummary_MultipleAnomalies(){
    DiagnosticSummary summary;

    std::vector<TimedPose> trajectory;

    trajectory.push_back({0.0, {0.1, 0.1, 0.05}});
    trajectory.push_back({0.1, {65.0, 0.15, 0.1}});
    trajectory.push_back({10.0, {5.0, 5.0, 0.15}});
    trajectory.push_back({10.0, {0.4, 0.25, 0.2}});
    trajectory.push_back({9.4, {0.3, 2.7, 0.25}});

    std::optional<std::vector<JumpAnomaly>> jump_anomalies = detectJumpAnomaly(trajectory);
    std::optional<std::vector<TimestampAnomaly>> time_anomalies = detectTimeAnomaly(trajectory);

    updateDiagnosticSummary(summary, jump_anomalies);
    updateDiagnosticSummary(summary, time_anomalies);

    if (summary.totalAnomalyCount == 7 && summary.jumpCount == 4 && summary.repeatedTimeCount == 1 && summary.backwardTimeCount == 1 && summary.oversizedTimeCount == 1){
        std::cout << "DiagnosticSummary_MultipleAnomalies test: passed" << std::endl;
    }
    else{
        std::cout << "DiagnosticSummary_MultipleAnomalies test: failed" << std::endl;
    }
}

int main(){
    std::cout << "ComputeTotalPathLength tests:\n" << std::endl;
    std::cout << "-----------------------------\n" << std::endl;
    testComputeTotalPathLength();
    testComputeTotalPathLength_SingleElement();
    testComputeTotalPathLength_Empty();

    std::cout << "\nDetectJumpAnomaly tests:\n" << std::endl;
    std::cout << "-----------------------------\n" << std::endl;
    testDetectJumpAnomaly_NoJumps();
    testDetectJumpAnomaly_SingleJump();
    testDetectJumpAnomaly_Empty();
    testDetectJumpAnomaly_OnePose();
    testDetectJumpAnomaly_Boundary();

    std::cout << "\nDetectTimeAnomaly tests:\n" << std::endl;
    std::cout << "-----------------------------\n" << std::endl;
    testDetectTimeAnomaly_NoAnomalies();
    testDetectTimeAnomaly_Empty();
    testDetectTimeAnomaly_SingleElement();
    testDetectTimeAnomaly_Repeated();
    testDetectTimeAnomaly_Backward();
    testDetectTimeAnomaly_Oversized();

    std::cout << "\nSummary tests:\n" << std::endl;
    std::cout << "-----------------------------\n" << std::endl;
    testDiagnosticSummary_NoAnomalies();
    testDiagnosticSummary_OneJumpAnomaly();
    testDiagnosticSummary_RepeatedTimeAnomaly();
    testDiagnosticSummary_BackwardTimeAnomaly();
    testDiagnosticSummary_OversizedTimeAnomaly();
    testDiagnosticSummary_MultipleAnomalies();
    
    return 0;
}
