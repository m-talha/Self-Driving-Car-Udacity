/**
 * @file config.hpp
 * 
 * @brief Configuration parameters and constants
 * 
 * @author: M Talha
 */

#pragma once

namespace PathPlanner {
    // Constants for lanes
    constexpr double kLaneWidth = 4.0;
    constexpr int kLeft = 0;
    constexpr int kMiddle = 1;
    constexpr int kRight = 2;
    // Other constants
    constexpr double kTimeStep = 0.02;        // timestep used by simulator
    constexpr double kSafetyGap = 30.0;       // min. distance to maintain between cars
    constexpr double kHorizonDistance = 30.0; // trajectory horizon distance

    // Ego car configuration
    int lane = kMiddle;
    double curr_speed = 0.0;
    double max_speed = 49.5 / 2.24;
    double max_accel = 0.224;
}