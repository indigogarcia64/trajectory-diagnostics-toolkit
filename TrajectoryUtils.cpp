#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "TrajectoryUtils.hpp"

void printTrajectory(const std::vector<TimedPose>& trajectory) {
    if (!trajectory.empty()){
        for (const auto& timed_pose : trajectory) {
            std::cout << "Timestamp: " << timed_pose.timestamp << "s\n"
                      << "x coordinate: " << timed_pose.pose.x << "m\n"
                      << "y coordinate: " << timed_pose.pose.y << "m\n"
                      << "theta coordinate: " << timed_pose.pose.theta << "rad\n"
                      << std::endl;
        }
    }
    else{
        std::cout << "Trajectory is empty!" << std::endl;
    }
}

void printOptionalMetric(const std::string& label, const std::optional<double>& value){
    if (value.has_value()){
        std::cout << label << ": " << value.value() << std::endl;
    }
    else{
        std::cout << label << ": unavailable" << std::endl;
    }
}

bool compareByX(const TimedPose& a, const TimedPose& b) {
    return a.pose.x < b.pose.x;
}

void shiftX(std::vector<TimedPose>& trajectory, double dx) {
    for (auto& timed_pose : trajectory) {
        timed_pose.pose.x += dx;
    }
}

std::optional<double> computeAverageX(const std::vector<TimedPose>& trajectory) {
    if (trajectory.empty()){
        return std::nullopt;
    }

    double sum = 0.0;

    for (const auto& timed_pose : trajectory) {
        sum += timed_pose.pose.x;
    }

    return sum / trajectory.size();
}

std::optional<double> computeAverageY(const std::vector<TimedPose>& trajectory) {
    if (trajectory.empty()){
        return std::nullopt;
    }

    double sum = 0.0;

    for (const auto& timed_pose : trajectory) {
        sum += timed_pose.pose.y;
    }

    return sum / trajectory.size();
}

std::optional<double> findMinX(const std::vector<TimedPose>& trajectory) {
    if (trajectory.empty()){
        return std::nullopt;
    }

    double min_x = trajectory[0].pose.x;

    for (const auto& timed_pose : trajectory) {
        if (timed_pose.pose.x < min_x) {
            min_x = timed_pose.pose.x;
        }
    }

    return min_x;
}

std::optional<double> findMaxX(const std::vector<TimedPose>& trajectory) {
    if (trajectory.empty()){
        return std::nullopt;
    }

    double max_x = trajectory[0].pose.x;

    for (const auto& timed_pose : trajectory) {
        if (timed_pose.pose.x > max_x) {
            max_x = timed_pose.pose.x;
        }
    }

    return max_x;
}

std::optional<double> computeTotalPathLength(const std::vector<TimedPose>& trajectory){
    if (trajectory.empty()){
        return std::nullopt;
    }

    double total_path_length = 0.0;
    double current_x = trajectory[0].pose.x;
    double current_y = trajectory[0].pose.y;
    double prev_x = 0.0;
    double prev_y = 0.0;
    double x_diff = 0.0;
    double y_diff = 0.0;

    for (const auto& timed_pose : trajectory){
        prev_x = current_x;
        prev_y = current_y;

        current_x = timed_pose.pose.x;
        current_y = timed_pose.pose.y;

        x_diff = current_x - prev_x;
        y_diff = current_y - prev_y;

        total_path_length += sqrt((x_diff * x_diff) + (y_diff * y_diff));
    }

    return total_path_length;
}