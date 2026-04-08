#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <optional>

#include "TrajectoryIO.hpp"

std::optional<std::vector<TimedPose>> loadTrajectoryFromCSV(const std::string& filename){
    std::ifstream file(filename);

    if (!file){
        return std::nullopt;
    }
    
    std::vector<TimedPose> trajectory;
    std::string line;
    
    if (!std::getline(file, line)){
        return std::nullopt;
    }
    
    //While there are still more lines of data,
    while (std::getline(file,line)){
        //grab each line.
        std::stringstream ss(line);

        std::string field;
        std::vector<std::string> fields;
        
        //While there are still more elements in the line,
        while (std::getline(ss, field, ',')){
            //grab each element and push it into the vector.
            fields.push_back(field);
        }

        //The TimedPose struct expects 4 fields. If extracted data is not 4 fields, return std::nullopt.
        if (fields.size() != 4){
            return std::nullopt;
        }
    
        //Assign each element to their respective pose fields
        TimedPose sample;
        sample.timestamp = std::stod(fields[0]);
        sample.pose.x = std::stod(fields[1]);
        sample.pose.y = std::stod(fields[2]);
        sample.pose.theta = std::stod(fields[3]);
    
        //Add the pose to the trajectory
        trajectory.push_back(sample);
    }

    return trajectory;
}
