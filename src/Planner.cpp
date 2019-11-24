//
// Created by vishnu on 11/23/19.
//

#include "Planner.hpp"
#include "Cost.hpp"

/**
 * @todo Replace switch statements with boost::FSM
 */
Planner::Planner() = default;

Planner::Planner(double refSpeed, int refLane) : ref_speed(refSpeed), ref_lane(refLane) {
    std::cout << "Planner Initialized..." << std::endl;
}

void
Planner::update(double cx, double cy, double cs, double cd, double cyaw, double cvel, int lanenum, double targetVel,
                double delta) {
    x = cx;
    y = cy;
    s = cs;
    d = cd;
    yaw = cyaw;
    speed = cvel;
    deltat = delta;

    ref_speed = targetVel;
    ref_lane = lanenum;

    resetPlannerInfo_();
}

void Planner::resetPlannerInfo_() {
    trajectory.lane_start = ref_lane;
    trajectory.lane_end = ref_lane;
    trajectory.target_speed = ref_speed;

    nextTarget.ref_v = ref_speed;
    nextTarget.lane = ref_lane;

    collisionInfo.front_collision_distance = {FREE_DISTANCE, FREE_DISTANCE, FREE_DISTANCE};
    collisionInfo.rear_collision_distance = {FREE_DISTANCE, FREE_DISTANCE, FREE_DISTANCE};
    collisionInfo.collion_velocity = {FREE_VEL, FREE_VEL, FREE_VEL};
}

void Planner::nextFSMState(const std::vector<std::vector<double>> &sensor_info) {
    std::vector<States> states = getSuccessors_();
    updateCollisionInfo_(sensor_info);

    States best_state = findBestState_(states, sensor_info);

    state = best_state;

    // Update the behavior
    updatePlannerInfo_(state);

    double collision_speed = collisionInfo.collion_velocity[ref_lane];
    double collision_dist = collisionInfo.front_collision_distance[ref_lane];

    if (collision_dist > SAFE_DIST && collision_speed > ref_speed && ref_speed < MAX_VEL) {
        nextTarget.ref_v += MAX_ACCL;
    } else if (collision_dist < 30 && ref_speed > collision_speed && ref_speed > 0) {
        nextTarget.ref_v -= MAX_DECEL;
    }

}

std::vector<States> Planner::getSuccessors_() {
    std::vector<States> states;

    states.emplace_back(KL);

    if (state == PLCL) {
        states.emplace_back(LCL);
        states.emplace_back(PLCL);
    } else if (state == PLCR) {
        states.emplace_back(LCR);
        states.emplace_back(PLCR);
    } else {
        if (d < (2 + LANE_WIDTH * (ref_lane) + 2) && d > (2 + LANE_WIDTH * (ref_lane) - 2) && speed > 20) {
            if (ref_lane != 0) states.emplace_back(PLCL);
            if (ref_lane != 2) states.emplace_back(PLCR);
        }
    }

    return states;
}

void Planner::updateCollisionInfo_(const std::vector<std::vector<double>> &sensor_info) {
    for (const auto &traffic_info : sensor_info) {
        float traffic_d = traffic_info[6]; // d of current car (other traffic participant) being considered

        // In-lane traffic
        if (traffic_d < (2 + LANE_WIDTH * (double) ref_lane + 2) &&
            traffic_d > (2 + LANE_WIDTH * (double) ref_lane - 2)) {
            double vx = traffic_info[3];
            double vy = traffic_info[LANE_WIDTH];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = traffic_info[5];
            check_car_s += (double) deltat * check_speed;

            if ((check_car_s > s) && (check_car_s - s < SAFE_DIST)) {
                collisionInfo.front_collision_distance[ref_lane] = (check_car_s - s);
                collisionInfo.collion_velocity[ref_lane] = check_speed;
            } else if ((check_car_s < s) && (s - check_car_s < SAFE_DIST)) {
                collisionInfo.rear_collision_distance[ref_lane] = (s - check_car_s);
            }
        }/*Left-lane Traffic*/else if (traffic_d < ((LANE_WIDTH * (double) ref_lane))) {
            // get lane of the current traffic participant being considered
            int lane_num = (traffic_d < ((double) ref_lane - 1) * LANE_WIDTH) ? ref_lane - 2 : ref_lane - 1;
            if (lane_num > 2) std::cout << lane_num << std::endl;
            double vx = traffic_info[3];
            double vy = traffic_info[LANE_WIDTH];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = traffic_info[5];

            // Predict distance, given current speed (constant velocity model)
            check_car_s += (double) deltat * check_speed;
            if ((check_car_s - s) < 0) { // car at left lane behind us
                if ((s - check_car_s) > LANE_CHANGE_THRESHOLD)
                    continue; // collision will not happen,safe to change lane
                else collisionInfo.rear_collision_distance[lane_num] = (s - check_car_s);
            } else if ((check_car_s - s) < SAFE_DIST) { // car at left lane is ahead us
                collisionInfo.front_collision_distance[lane_num] = (check_car_s - s);
                collisionInfo.collion_velocity[lane_num] = check_speed;
            }
        } /*Right-lane traffic*/else if (traffic_d > (2 + 4 * (double)ref_lane + 2)) {

            int lane_num = (traffic_d > ((double) ref_lane + 2) * LANE_WIDTH) ? ref_lane + 2 : ref_lane + 1;

            if(lane_num > 2)
                std::cout << "lane number - " << lane_num << std::endl;
            double vx = traffic_info[3];
            double vy = traffic_info[LANE_WIDTH];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = traffic_info[5];

            check_car_s += (double) deltat * check_speed;
            if ((check_car_s - s) < 0) { // car at right lane behind us
                if ((s - check_car_s) > LANE_CHANGE_THRESHOLD)
                    continue; // collision will not happen,safe to change lane
                else collisionInfo.rear_collision_distance[lane_num] = (s - check_car_s);
            } else if ((check_car_s - s) < SAFE_DIST) {
                collisionInfo.front_collision_distance[lane_num] = (check_car_s - s);
                collisionInfo.collion_velocity[lane_num] = check_speed;
            }
        }

    }
}

States Planner::findBestState_(std::vector<States> states, const std::vector<std::vector<double>> &sensor_info) {

    double cost;
    States best_state = KL;

    double best_cost = std::numeric_limits<double>::max();

    for (auto &state_ : states) {
        updatePlannerInfo_(state_);

        // Decision depends on the state of the vehicle and traffic participants
        Cost CF = Cost(this, sensor_info);
        cost = CF.calculateCost();
        if (cost < best_cost) {
            best_cost = cost;
            best_state = state_;
        }
    }
    return best_state;
}

void Planner::updatePlannerInfo_(States &state_) {
    switch (state_) {
        case KL: {
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane;
            nextTarget.lane = ref_lane;
            break;
        }
        case PLCL: {
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane - 1;
            nextTarget.lane = ref_lane;
            break;
        }

        case LCL: {
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane - 1;
            nextTarget.lane = ref_lane - 1;
            break;
        }

        case PLCR: {
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane + 1;
            nextTarget.lane = ref_lane;
            break;
        }

        case LCR: {
            trajectory.lane_start = ref_lane;
            trajectory.lane_end = ref_lane + 1;
            nextTarget.lane = ref_lane + 1;
            break;
        }
        default:
            // Execute emergency stop or fall back maneuver
//            throw std::runtime_error("STATE ERROR");
            std::cout << "[WARN] STATE ERROR\n ";
    }
}

