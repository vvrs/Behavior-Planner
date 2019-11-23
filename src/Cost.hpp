//
// Created by vishnu on 11/23/19.
//

#pragma once


#include <cmath>
#include <vector>
#include "Planner.hpp"

class Cost {
public:
    const double DANGER = pow(10, 7);
    const double REACH_GOAL = pow(10, 5);
    const double EFFICIENCY = pow(10, 5);
    const double COMFORT = pow(10, 4);

    std::vector<std::vector<double>> sensor_info;
    Planner* planner;

    Cost(Planner* planner_, std::vector<std::vector<double>> s);

    double calculateCost();

    double reachGoalCost();

    double inefficiencyCost();

    double safeDistanceCost();

    double comfortCost();
};



