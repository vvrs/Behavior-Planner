//
// Created by vishnu on 11/23/19.
//

#pragma once


#include <cmath>
#include <vector>
#include "Planner.hpp"
#include "utils.hpp"

class Cost {
public:
    const double SAFE_DISTANCE_COST = pow(10, 7);
    const double REACH_GOAL_COST = pow(10, 5);
    const double EFFICIENCY_COST = pow(10, 5);
    const double COMFORT_COST = pow(10, 4);

    std::vector<std::vector<double>> sensor_info;
    Planner* planner;

    Cost(Planner* planner_, std::vector<std::vector<double>> s);

    double calculateCost();

    double reachGoalCost();

    double inefficiencyCost();

    double safeDistanceCost();

    double comfortCost();
};



