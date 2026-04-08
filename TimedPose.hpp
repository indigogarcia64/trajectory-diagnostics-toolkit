#pragma once

struct Pose2D{
    double x;
    double y;
    double theta;
};

struct TimedPose{
    double timestamp;
    Pose2D pose;
};