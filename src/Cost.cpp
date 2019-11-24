//
// Created by vishnu on 11/23/19.
//

#include "Cost.hpp"


Cost::Cost(Planner *planner_, std::vector<std::vector<double>> s) {
    planner = planner_;
    sensor_info = std::move(s);
}

double Cost::calculateCost() {
    return reachGoalCost() + inefficiencyCost() + safeDistanceCost() + comfortCost();
}

double Cost::reachGoalCost() {
    int end_lane = planner->trajectory.lane_end;
    return exp(-planner->collisionInfo.front_collision_distance[end_lane] / 50) * REACH_GOAL_COST;
}

double Cost::inefficiencyCost() {
    int end_lane = planner->trajectory.lane_end;
    return ((49.5 - planner->collisionInfo.collion_velocity[end_lane]) / 49.5) *
           ((49.5 - planner->collisionInfo.collion_velocity[end_lane]) / 49.5) * EFFICIENCY_COST;
}

double Cost::safeDistanceCost() {
    int end_lane = planner->trajectory.lane_end;
    int start_lane = planner->trajectory.lane_start;
    if (end_lane != start_lane) { // if about to change a lane
        return (exp(-planner->collisionInfo.rear_collision_distance[end_lane] / 100) +
                exp(-planner->collisionInfo.front_collision_distance[end_lane] / 100)) * SAFE_DISTANCE_COST;
    }
    return 0; // if stay in the lane
}

double Cost::comfortCost() {
    int start_lane = planner->trajectory.lane_start;
    int end_lane = planner->trajectory.lane_end;
    return (start_lane != end_lane) * COMFORT_COST;
}