#include <iostream>
#include <cmath>

#include "TrajectoryDiagnostics.hpp"

void printAnomalies(const std::vector<JumpAnomaly>& anomalies){
    for (const auto& anomaly : anomalies){
        std::cout << "Anomaly type: Jump\n"
                  << "Occurred between: " << anomaly.prev_timestamp << "s and " << anomaly.curr_timestamp << "s\n"
                  << "Jump distance: " << anomaly.distance << "m\n"
                  << std::endl;
    }
}

void printAnomalies(const std::vector<TimestampAnomaly>& anomalies){
    for (const auto& anomaly : anomalies){
        switch(anomaly.type){
            case (TimestampAnomalyType::Repeated):
                std::cout << "Anomaly type: Time - Repeated" << std::endl;
                break;

            case (TimestampAnomalyType::Backward):
                std::cout << "Anomaly type: Time - Backward" << std::endl;
                break;

            case (TimestampAnomalyType::Oversized):
                std::cout << "Anomaly type: Time - Oversized" << std::endl;
                break;

            default:
                break;
        }

        std::cout << "Occurred between: " << anomaly.prev_timestamp << "s and " << anomaly.curr_timestamp << "s\n"
                  << "Time gap: " << anomaly.time_gap << "s\n"
                  << std::endl;
    }
}

void printSummary(const DiagnosticSummary& summary){
    std::cout << "DIAGNOSTIC SUMMARY" << "\n"
              << "---------------------------------------------------------" << "\n"
              << "Total anomaly count: " << summary.totalAnomalyCount << "\n"
              << "Jump count: " << summary.jumpCount << "\n"
              << "Repeated time count: " << summary.repeatedTimeCount << "\n"
              << "Backward time count: " << summary.backwardTimeCount << "\n"
              << "Oversized time count: " << summary.oversizedTimeCount << "\n"
              << std::endl;
}

void reportAnomalies(const std::optional<std::vector<JumpAnomaly>>& anomalies) {
    if (anomalies == std::nullopt){
        std::cout << "\nEmpty trajectory!\n" << std::endl;
        return;
    }

    if (anomalies->empty()) {
        std::cout << "\nNo jump anomalies detected.\n" << std::endl;
        return;
    }

    std::cout << "\nJump anomalies detected!\n" << std::endl;
    printAnomalies(*anomalies);
}

void reportAnomalies(const std::optional<std::vector<TimestampAnomaly>>& anomalies) {
    if (anomalies == std::nullopt){
        std::cout << "\nEmpty trajectory!\n" << std::endl;
        return;
    }

    if (anomalies->empty()) {
        std::cout << "\nNo time anomalies detected.\n" << std::endl;
        return;
    }

    std::cout << "\nTime anomalies detected!\n" << std::endl;
    printAnomalies(*anomalies);
}

void updateDiagnosticSummary(DiagnosticSummary& summary, const std::optional<std::vector<JumpAnomaly>>& jumpAnomalies){
    if (jumpAnomalies){
        for (const auto& anomaly : *jumpAnomalies){
            summary.jumpCount++;
            summary.totalAnomalyCount++;
        }
    }
}

void updateDiagnosticSummary(DiagnosticSummary& summary, const std::optional<std::vector<TimestampAnomaly>>& timeAnomalies){
    if (timeAnomalies){
        for (const auto& anomaly : *timeAnomalies){
            summary.totalAnomalyCount++;
            
            if (anomaly.type == TimestampAnomalyType::Repeated){
                summary.repeatedTimeCount++;
            }
            
            if (anomaly.type == TimestampAnomalyType::Backward){
                summary.backwardTimeCount++;
            }

            if (anomaly.type == TimestampAnomalyType::Oversized){
                summary.oversizedTimeCount++;
            }
        }
    }
}

std::optional<std::vector<JumpAnomaly>> detectJumpAnomaly(const std::vector<TimedPose>& trajectory){
    if (trajectory.empty()){
        return std::nullopt;
    }

    std::vector<JumpAnomaly> anomalies;

    double path_segment;
    double current_timestamp;
    double current_x;
    double current_y;
    double prev_timestamp;
    double prev_x;
    double prev_y;
    double x_diff;
    double y_diff;
    
    for (std::size_t i = 1; i < trajectory.size(); i++){

        current_timestamp = trajectory[i].timestamp;
        current_x = trajectory[i].pose.x;
        current_y = trajectory[i].pose.y;

        prev_timestamp = trajectory[i-1].timestamp;
        prev_x = trajectory[i-1].pose.x;
        prev_y = trajectory[i-1].pose.y;

        x_diff = current_x - prev_x;
        y_diff = current_y - prev_y;

        path_segment = sqrt((x_diff * x_diff) + (y_diff * y_diff));

        if (path_segment > JUMP_THRESHOLD){
            JumpAnomaly anomaly;

            anomaly.curr_timestamp = current_timestamp;
            anomaly.prev_timestamp = prev_timestamp;
            anomaly.distance = path_segment;

            anomalies.push_back(anomaly);
        }
    }

    return anomalies;
}

std::optional<std::vector<TimestampAnomaly>> detectTimeAnomaly(const std::vector<TimedPose>& trajectory){
    if (trajectory.empty()){
        return std::nullopt;
    }

    std::vector<TimestampAnomaly> anomalies;

    TimestampAnomalyType type;
    bool detected_anomaly = false;
    double current_timestamp;
    double prev_timestamp;

    for (std::size_t i = 1; i < trajectory.size(); i++){
        current_timestamp = trajectory[i].timestamp;
        prev_timestamp = trajectory[i-1].timestamp;
        
        if (current_timestamp == prev_timestamp){
            detected_anomaly = true;
            type = TimestampAnomalyType::Repeated;
        }

        else if (current_timestamp < prev_timestamp){
            detected_anomaly = true;
            type = TimestampAnomalyType::Backward;
        }

        else if ((current_timestamp - prev_timestamp) > TIME_JUMP_THRESHOLD){
            detected_anomaly = true;
            type = TimestampAnomalyType::Oversized;
        }

        if (detected_anomaly){
            detected_anomaly = false;

            TimestampAnomaly anomaly;

            anomaly.curr_timestamp = current_timestamp;
            anomaly.prev_timestamp = prev_timestamp;
            anomaly.time_gap = current_timestamp - prev_timestamp;
            anomaly.type = type;

            anomalies.push_back(anomaly);
        }
    }

    return anomalies;
}