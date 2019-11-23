//
// Created by vishnu on 11/23/19.
//

#pragma once

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

constexpr double DT = .02;
constexpr double FREE_DISTANCE = 999.;
constexpr double FREE_VEL = 49.5;
constexpr double MAX_VEL = 49.5;
constexpr double SAFE_DIST = 30.;
constexpr double MAX_ACCL = .224;
constexpr double MAX_DECEL = .224;
constexpr int LANE_WIDTH = 4;
constexpr int LANE_CHANGE_THRESHOLD = 15;
/**
 * KL - Keep Lane
 * LCL - Lane Change Left
 * LCR - Lane Change Right
 * PLCL - Prepare for LCL
 * PLCR - Prepare for LCR
 */

enum States {
    KL, LCL, LCR, PLCL, PLCR
};

class Planner {
public:
    Planner();

    Planner(double refSpeed, int refLane);

    States state = KL; // initial state in FSM
    double ref_speed = 0;
    int ref_lane = 0;

    double x = 0;
    double y = 0;
    double s = 0;
    double d = 0;
    double yaw = 0;
    double speed = 0;
    double deltat = 0;


    struct trajectory {
        int lane_start = 1;
        int lane_end = 1;
        double target_speed = 0;
    } trajectory;


    struct nextTarget {
        double ref_v = 0;
        int lane = 0;
    } nextTarget;


    struct collisionInfo {
        /**
         * @struct collisionInfo
         * Keeps distance and speeds of vehicles in the right, current, and left lanes
         */
        std::vector<double> dist_to_colli_front = {FREE_DISTANCE, FREE_DISTANCE, FREE_DISTANCE};
        std::vector<double> dist_to_colli_back = {FREE_DISTANCE, FREE_DISTANCE, FREE_DISTANCE};
        std::vector<double> spd_of_colli = {FREE_VEL, FREE_VEL, FREE_VEL};
    } collisionInfo;

    void update(double cx, double cy, double cs, double cd, double cyaw, double cvel, int lanenum, double targetVel,
                double delta);


    void nextFSMState(const std::vector<std::vector<double>> &senfor_info);

private:
    void resetPlannerInfo_();

    std::vector<States> getSuccessors_();

    void updateCollisionInfo_(const std::vector<std::vector<double>> &sensor_info);

    States findBestState_(std::vector<States> states, const std::vector<std::vector<double>> &sensor_info);

    /**
     * @brief Method to make a decision and update lane and reference speed correspondingly
     * @param state_
     */
    void updatePlannerInfo_(States &state_);
};



