/***************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file crash_land.hpp
 * This file defines structures for deliberate crash landing approaches for fixed-wing aircraft.
 * It provides parameters for steep descent profiles designed to impact a target with maximum force.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <geo/geo.h>

/**
 * @brief Defines a crash landing approach for fixed-wing aircraft
 *
 * This structure contains all parameters needed to execute a steep descent
 * crash landing, including entry point, dive parameters, and target point.
 */
struct crash_point_s {
    // Entry point parameters
    double entry_lat;            // Latitude of approach entry point
    double entry_lon;            // Longitude of approach entry point
    float entry_altitude_m;      // Altitude at entry point

    float entry_airspeed_ms;     // Target airspeed at entry

    // Dive parameters
    float dive_angle_deg;        // Dive angle (negative, in degrees)
    float dive_airspeed_ms;      // Target airspeed during dive
    float max_dive_airspeed_ms;  // Maximum allowable airspeed during dive
    float throttle_setting;      // Throttle setting during dive (0.0-1.0)

    // Target point
    double target_lat;           // Target point latitude
    double target_lon;           // Target point longitude

    // Optional parameters
    float terminal_maneuver;     // Optional terminal maneuver (0=none, 1=pitch down, 2=roll)

    crash_point_s()
    {
        reset();
    }

    void reset()
    {
        entry_lat = entry_lon = target_lat = target_lon = static_cast<double>(NAN);
        entry_altitude_m = entry_airspeed_ms = dive_angle_deg = dive_airspeed_ms = NAN;
        max_dive_airspeed_ms = throttle_setting = terminal_maneuver = NAN;
    }

    bool isValid() const
    {
        return PX4_ISFINITE(entry_lat) && PX4_ISFINITE(entry_lon) &&
               PX4_ISFINITE(target_lat) && PX4_ISFINITE(target_lon) &&
               PX4_ISFINITE(entry_altitude_m) && PX4_ISFINITE(dive_angle_deg) &&
               PX4_ISFINITE(dive_airspeed_ms) && PX4_ISFINITE(max_dive_airspeed_ms);
    }

    /**
     * @brief Calculate the total approach distance
     *
     * @return float Distance in meters from entry point to target point
     */
    float getTotalApproachDistance() const
    {
        if (!isValid()) {
            return -INFINITY;
        }

        return get_distance_to_next_waypoint(entry_lat, entry_lon, target_lat, target_lon);
    }

    /**
     * @brief Calculate the estimated time for the approach
     *
     * @return float Time in seconds for the complete approach
     */
    float getEstimatedApproachTime() const
    {
        if (!isValid()) {
            return -INFINITY;
        }

        // Distance from entry to target
        float dist_to_target = getTotalApproachDistance();

        // Average speed during dive (simplified)
        float avg_dive_speed = (entry_airspeed_ms + dive_airspeed_ms) / 2.0f;

        // Time for dive phase
        return dist_to_target / avg_dive_speed;
    }

    /**
     * @brief Calculate the estimated impact velocity
     *
     * @return float Estimated impact velocity in m/s
     */
    float getEstimatedImpactVelocity() const
    {
        if (!isValid()) {
            return -INFINITY;
        }

        // Simple ballistic model - could be enhanced with aerodynamic modeling
        float horizontal_velocity = dive_airspeed_ms * cosf(fabsf(dive_angle_deg) * static_cast<float>(M_DEG_TO_RAD));
        float vertical_velocity = dive_airspeed_ms * sinf(fabsf(dive_angle_deg) * static_cast<float>(M_DEG_TO_RAD));

        // Add gravitational acceleration component
        float time_to_impact = getEstimatedApproachTime();
        vertical_velocity += 9.81f * time_to_impact;

        // Calculate resultant velocity
        return sqrtf(horizontal_velocity * horizontal_velocity + vertical_velocity * vertical_velocity);
    }

    /**
     * @brief Calculate the estimated impact energy
     *
     * @param aircraft_mass Aircraft mass in kg
     * @return float Estimated impact energy in Joules
     */
    float getEstimatedImpactEnergy(float aircraft_mass) const
    {
        if (!isValid() || aircraft_mass <= 0.0f) {
            return -INFINITY;
        }

        float impact_velocity = getEstimatedImpactVelocity();
        return 0.5f * aircraft_mass * impact_velocity * impact_velocity;
    }
};

/**
 * @brief Container for multiple crash approach options
 *
 * Allows defining multiple approach options for different target conditions
 * or other operational requirements.
 */
struct crash_approaches_s {
    static constexpr uint8_t num_approaches_max = 4;
    crash_point_s approaches[num_approaches_max];

    crash_approaches_s()
    {
        resetAllApproaches();
    }

    void resetAllApproaches()
    {
        for (uint8_t i = 0; i < num_approaches_max; i++) {
            approaches[i].reset();
        }
    }

    bool isAnyApproachValid() const
    {
        for (uint8_t i = 0; i < num_approaches_max; i++) {
            if (approaches[i].isValid()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Find the best approach for current wind conditions
     *
     * @param wind_speed Current wind speed in m/s
     * @param wind_direction Wind direction in radians
     * @return int8_t Index of best approach or -1 if none suitable
     */
    int8_t getBestApproachIndex(float wind_speed, float wind_direction) const
    {
        if (!isAnyApproachValid() || wind_speed < 0.0f) {
            return -1;
        }

        int8_t best_idx = -1;
        float best_score = -INFINITY;

        for (uint8_t i = 0; i < num_approaches_max; i++) {
            if (approaches[i].isValid()) {
                // Calculate approach direction
                float approach_direction = get_bearing_to_next_waypoint(
                    approaches[i].entry_lat, approaches[i].entry_lon,
                    approaches[i].target_lat, approaches[i].target_lon);

                // Calculate headwind component (higher is better for accuracy)
                float wind_angle_diff = matrix::wrap_pi(approach_direction - wind_direction);
                float headwind_component = wind_speed * cosf(wind_angle_diff);

                // Simple scoring: prefer approaches with higher headwind component
                float score = headwind_component;

                if (score > best_score) {
                    best_score = score;
                    best_idx = i;
                }
            }
        }

        return best_idx;
    }
};
