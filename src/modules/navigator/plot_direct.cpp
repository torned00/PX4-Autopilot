/****************************************************************************
 *
 *   Copyright (c) 2013-2024 PX4 Development Team. All rights reserved.
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
 * @file plot_direct.cpp
 *
 * Class to access PLOT
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#include <float.h>

#include "plot_direct.h"
#include "navigator.h"
#include <px4_platform_common/events.h>

#include <lib/geo/geo.h>

using namespace math;

PlotDirect::PlotDirect(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PLOT),
	ModuleParams(navigator)
{
	_destination.lat = static_cast<double>(NAN);
	_destination.lon = static_cast<double>(NAN);
	_land_approach.lat = static_cast<double>(NAN);
	_land_approach.lon = static_cast<double>(NAN);
	_land_approach.height_m = NAN;
}

void PlotDirect::on_inactivation()
{
	_plot_state = PLOTState::IDLE;
}

void PlotDirect::on_activation()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	_plot_state = getActivationLandState();

	// reset cruising speed and throttle to default for PLOT
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	set_plot_item();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "PLOT: start return at %d m (%d m above destination)\t",
			 (int)ceilf(_plot_alt), (int)ceilf(_plot_alt - _destination.alt));
	events::send<int32_t, int32_t>(events::ID("vplot_return_at"), events::Log::Info,
				       "PLOT: start return at {1m_v} ({2m_v} above destination)",
				       (int32_t)ceilf(_plot_alt), (int32_t)ceilf(_plot_alt - _destination.alt));
}

void PlotDirect::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();

	parameters_update();

	if (_plot_state != PLOTState::IDLE && is_mission_item_reached_or_completed()) {
		_updatePlotState();
		set_plot_item();
	}

}

void PlotDirect::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
}

void PlotDirect::setPlotPosition(PositionYawSetpoint plot_position, loiter_point_s loiter_pos)
{
	_home_pos_sub.update();

	parameters_update();

	// Only allow to set a new approach if the mode is not activated yet.
	if (!isActive()) {
		_destination = plot_position;
		_force_heading = false;

		// Input sanitation
		if (!PX4_ISFINITE(_destination.lat) || !PX4_ISFINITE(_destination.lon)) {
			// We don't have a valid plot position, use the home position instead.
			_destination.lat = _home_pos_sub.get().lat;
			_destination.lon = _home_pos_sub.get().lon;
			_destination.alt = _home_pos_sub.get().alt;
			_destination.yaw = _home_pos_sub.get().yaw;
		}

		if (!PX4_ISFINITE(_destination.alt)) {
			// Not a valid plot land altitude. Assume same altitude as home position.
			_destination.alt = _home_pos_sub.get().alt;
		}

		_land_approach = sanitizeLandApproach(loiter_pos);

		const float dist_to_destination{get_distance_to_next_waypoint(_land_approach.lat, _land_approach.lon, _destination.lat, _destination.lon)};

		if (dist_to_destination > _navigator->get_acceptance_radius()) {
			_force_heading = true;
		}
	}
}

void PlotDirect::_updatePlotState()
{
	PLOTState new_state{PLOTState::IDLE};

	switch (_plot_state) {

	case PLOTState::MOVE_TO_TARGET:
		new_state = PLOTState::MOVE_TO_LAND;
		break;

	case PLOTState::MOVE_TO_LAND:
		new_state = PLOTState::HIT_TARGET;
		break;

	case PLOTState::HIT_TARGET:
		new_state = PLOTState::IDLE;
		break;

	case PLOTState::IDLE: // Fallthrough
	default:
		new_state = PLOTState::IDLE;
		break;
	}

	_plot_state = new_state;
}


void PlotDirect::set_plot_item()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
				       _global_pos_sub.get().lat, _global_pos_sub.get().lon);

	const float loiter_altitude = math::min(_land_approach.height_m, _plot_alt);

	const bool is_close_to_destination = destination_dist < _param_plot_min_dist.get();

	float altitude_acceptance_radius = static_cast<float>(NAN);

	switch (_plot_state) {

	case PLOTState::MOVE_TO_TARGET: {
			PositionYawSetpoint pos_yaw_sp {
				.lat = _land_approach.lat,
				.lon = _land_approach.lon,
				.alt = _plot_alt,
			};

			// For FW flight:set to LOITER_TIME (with 0s loiter time), such that the loiter (orbit) status
			// can be displayed on groundstation and the WP is accepted once within loiter radius
			if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
				pos_yaw_sp.yaw = NAN;
				setLoiterHoldMissionItem(_mission_item, pos_yaw_sp, 0.f, _land_approach.loiter_radius_m);

			} else {
				// already set final yaw if close to destination and weather vane is disabled
				pos_yaw_sp.yaw = (is_close_to_destination && !_param_wv_en.get()) ? _destination.yaw : NAN;
				setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);
			}

			break;
		}


	case PLOTState::MOVE_TO_LAND: {

			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.alt = loiter_altitude;
			pos_yaw_sp.yaw = NAN;

			setMoveToPositionMissionItem(_mission_item, pos_yaw_sp);


			// set previous item location to loiter location such that vehicle tracks line between loiter
			// location and land location after exiting the loiter circle
			pos_sp_triplet->previous.lat = _land_approach.lat;
			pos_sp_triplet->previous.lon = _land_approach.lon;
			pos_sp_triplet->previous.alt = get_absolute_altitude_for_item(_mission_item);
			pos_sp_triplet->previous.valid = true;

			break;
		}

	case PLOTState::HIT_TARGET: {
			PositionYawSetpoint pos_yaw_sp{_destination};
			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled
			setLandMissionItem(_mission_item, pos_yaw_sp);


			mavlink_log_info(_navigator->get_mavlink_log_pub(), "PLOT: land at destination\t");
			events::send(events::ID("plot_land_at_destination"), events::Log::Info, "PLOT: land at destination");
			break;
		}

	case PLOTState::IDLE: {
			set_idle_item(&_mission_item);
			_navigator->mode_completed(getNavigatorStateId());
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!MissionBlock::item_contains_position(_mission_item)) {
		issue_command(_mission_item);

	} else {
		// Convert mission item to current position setpoint and make it valid.
		if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
			pos_sp_triplet->current.alt_acceptance_radius = altitude_acceptance_radius;
			_navigator->set_position_setpoint_triplet_updated();
		}
	}

	publish_plot_direct_navigator_mission_item(); // for logging
}

PlotDirect::PLOTState PlotDirect::getActivationLandState()
{
	_land_detected_sub.update();

	PLOTState land_state;

	land_state = PLOTState::MOVE_TO_TARGET;

	return land_state;
}

rtl_time_estimate_s PlotDirect::calc_rtl_time_estimate()
{
	_global_pos_sub.update();
	_plot_time_estimator.update();
	_plot_time_estimator.setVehicleType(_vehicle_status_sub.get().vehicle_type);
	_plot_time_estimator.reset();

	PLOTState start_state_for_estimate;

	if (isActive()) {
		start_state_for_estimate = _plot_state;

	} else {
		start_state_for_estimate = getActivationLandState();
	}

	// Calculate PLOT time estimate only when there is a valid destination
	// TODO: Also check if vehicle position is valid
	if (PX4_ISFINITE(_destination.lat) && PX4_ISFINITE(_destination.lon) && PX4_ISFINITE(_destination.alt)) {

		loiter_point_s land_approach = sanitizeLandApproach(_land_approach);

		const float loiter_altitude = min(land_approach.height_m, _plot_alt);

		// Sum up time estimate for various segments of the landing procedure
		switch (start_state_for_estimate) {

		// FALLTHROUGH
		case PLOTState::MOVE_TO_TARGET: {
				matrix::Vector2f direction{};
				get_vector_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, land_approach.lat,
							    land_approach.lon, &direction(0), &direction(1));
				float move_to_land_dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, land_approach.lat, land_approach.lon)};

				if (_vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) {
					move_to_land_dist = max(0.f, move_to_land_dist - land_approach.loiter_radius_m);
				}

				_plot_time_estimator.addDistance(move_to_land_dist, direction, 0.f);
			}

		// FALLTHROUGH
		case PLOTState::MOVE_TO_LAND:
		case PLOTState::HIT_TARGET: {
				float initial_altitude;

				// Add land segment (second landing phase) which comes after LOITER
				if (start_state_for_estimate == PLOTState::HIT_TARGET) {
					// If we are in this phase, use the current vehicle altitude  instead
					// of the altitude paramteter to get a continous time estimate
					initial_altitude = _global_pos_sub.get().alt;


				} else {
					// If this phase is not active yet, simply use the loiter altitude,
					// which is where the LAND phase will start
					initial_altitude = loiter_altitude;
				}

				if (_vehicle_status_sub.get().is_vtol) {
					_plot_time_estimator.setVehicleType(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				}

				_plot_time_estimator.addVertDistance(_destination.alt - initial_altitude);
			}

			break;

		case PLOTState::IDLE:
			// Remaining time is 0
			break;
		}
	}

	return _plot_time_estimator.getEstimate();
}

void PlotDirect::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

loiter_point_s PlotDirect::sanitizeLandApproach(loiter_point_s land_approach) const
{
	loiter_point_s sanitized_land_approach{land_approach};

	if (!PX4_ISFINITE(land_approach.lat) || !PX4_ISFINITE(land_approach.lon)) {
		sanitized_land_approach.lat = _destination.lat;
		sanitized_land_approach.lon = _destination.lon;
	}

	if (!PX4_ISFINITE(land_approach.height_m)) {
		sanitized_land_approach.height_m = _destination.alt + _param_plot_descend_alt.get();
	}

	if (!PX4_ISFINITE(land_approach.loiter_radius_m) || fabsf(land_approach.loiter_radius_m) <= FLT_EPSILON) {
		sanitized_land_approach.loiter_radius_m = _param_plot_loiter_rad.get();
	}

	return sanitized_land_approach;
}

void PlotDirect::publish_plot_direct_navigator_mission_item()
{
	navigator_mission_item_s navigator_mission_item{};

	navigator_mission_item.sequence_current = static_cast<uint16_t>(_plot_state);
	navigator_mission_item.nav_cmd = _mission_item.nav_cmd;
	navigator_mission_item.latitude = _mission_item.lat;
	navigator_mission_item.longitude = _mission_item.lon;
	navigator_mission_item.altitude = _mission_item.altitude;

	navigator_mission_item.time_inside = get_time_inside(_mission_item);
	navigator_mission_item.acceptance_radius = _mission_item.acceptance_radius;
	navigator_mission_item.loiter_radius = _mission_item.loiter_radius;
	navigator_mission_item.yaw = _mission_item.yaw;

	navigator_mission_item.frame = _mission_item.frame;
	navigator_mission_item.frame = _mission_item.origin;

	navigator_mission_item.loiter_exit_xtrack = _mission_item.loiter_exit_xtrack;
	navigator_mission_item.force_heading = _mission_item.force_heading;
	navigator_mission_item.altitude_is_relative = _mission_item.altitude_is_relative;
	navigator_mission_item.autocontinue = _mission_item.autocontinue;
	navigator_mission_item.vtol_back_transition = _mission_item.vtol_back_transition;

	navigator_mission_item.timestamp = hrt_absolute_time();

	_navigator_mission_item_pub.publish(navigator_mission_item);
}
