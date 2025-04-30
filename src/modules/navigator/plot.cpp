/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file plot.cpp
 *
 * Class to access PLOT
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#include <float.h>

#include "plot.h"
#include "navigator.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

#include <lib/geo/geo.h>

using namespace time_literals;
using namespace math;
using matrix::wrap_pi;

static constexpr float MIN_DIST_THRESHOLD = 2.f;

PLOT::PLOT(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PLOT),
	ModuleParams(navigator)
{
	_destination.lat = static_cast<double>(NAN);
	_destination.lon = static_cast<double>(NAN);

	_crash_approach.entry_lat = static_cast<double>(NAN);
	_crash_approach.entry_lon = static_cast<double>(NAN);
	_crash_approach.entry_altitude_m = NAN;
	_crash_approach.entry_radius_m = _param_plot_loiter_rad.get();
}

void PLOT::on_inactivation()
{
	_plot_state = PLOTState::IDLE;
}

void PLOT::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	parameters_update();

	// Limit inactive calculation to 0.5Hz
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 2_s) {
		_destination_check_time = now;
		setPlotTypeAndDestination();
		publishRemainingTimeEstimate();
	}

}

void PLOT::publishRemainingTimeEstimate()
{
	const bool global_position_recently_updated = _global_pos_sub.get().timestamp > 0
			&& hrt_elapsed_time(&_global_pos_sub.get().timestamp) < 10_s;

	rtl_time_estimate_s estimated_time{};
	estimated_time.valid = false;

	if (_navigator->home_global_position_valid() && global_position_recently_updated) {
		estimated_time = calc_rtl_time_estimate();
	}

	_rtl_time_estimate_pub.publish(estimated_time);
}

void PLOT::on_activation()
{
	setPlotTypeAndDestination();

	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_land_detected_sub.update();

	parameters_update();

	_plot_state = PLOTState::MOVE_TO_TARGET;

	// reset cruising speed and throttle to default for PLOT
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	setPlotAltMin(_enforce_plot_alt);

	set_plot_item();

	mavlink_log_info(_navigator->get_mavlink_log_pub(), "PLOT: start glide at %d m (%d m above destination)\t",
			 (int)ceilf(_plot_alt), (int)ceilf(_plot_alt - _destination.alt));
	events::send<int32_t, int32_t>(events::ID("plot_glide_to_target"), events::Log::Info,
				       "PLOT: start glide at {1m_v} ({2m_v} above destination)",
				       (int32_t)ceilf(_plot_alt), (int32_t)ceilf(_plot_alt - _destination.alt));



}

void PLOT::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	parameters_update();

	if (_plot_state != PLOTState::IDLE && is_mission_item_reached_or_completed()) {
		_updatePlotState();
		set_plot_item();
	}

	// Keep publishing remaining time estimates every 2 seconds
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 2_s) {
		_destination_check_time = now;
		publishRemainingTimeEstimate();
	}
}

void PLOT::setPlotTypeAndDestination()
{

	if (_param_plot_type.get() != 2) {
		// check the closest allowed destination.
		PositionYawSetpoint plot_position;
		float plot_alt;
		findPlotDestination(plot_position, plot_alt);

		crash_point_s landing_pos;
		landing_pos.entry_lat = plot_position.lat;
		landing_pos.entry_lon = plot_position.lon;
		landing_pos.entry_altitude_m = plot_alt;

		_plot_type = PlotType::PLOT_DIRECT;
		setPlotAlt(plot_alt);
		setPlotPosition(plot_position, landing_pos);

	}

	// Publish plot status
	_plot_status_pub.get().timestamp = hrt_absolute_time();
	_plot_status_pub.get().rtl_type = static_cast<uint8_t>(_plot_type);
	_plot_status_pub.update();

}

void PLOT::findPlotDestination(PositionYawSetpoint &plot_position, float &plot_alt)
{
	// set destination to home per default, then check if other valid landing spot is closer
	plot_position.alt = _home_pos_sub.get().alt;
	plot_position.lat = _home_pos_sub.get().lat;
	plot_position.lon = _home_pos_sub.get().lon;
	plot_position.yaw = _home_pos_sub.get().yaw;

	plot_alt = _global_pos_sub.get().alt;
}

void PLOT::setPlotPosition(PositionYawSetpoint plot_position, crash_point_s crash_pos)
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

		_crash_approach = sanitizeCrashApproach(crash_pos);

		const float dist_to_destination{get_distance_to_next_waypoint(_crash_approach.entry_lat, _crash_approach.entry_lon, _destination.lat, _destination.lon)};

		if (dist_to_destination > _navigator->get_acceptance_radius()) {
			_force_heading = true;
		}
	}
}

void PLOT::_updatePlotState()
{
	PLOTState new_state{PLOTState::IDLE};

	switch (_plot_state) {

	case PLOTState::MOVE_TO_TARGET:
		new_state = PLOTState::TRANSITION_TO_DESCEND;
		break;

	case PLOTState::TRANSITION_TO_DESCEND:
		new_state = PLOTState::STEEP_DESCENT;
		break;

	case PLOTState::STEEP_DESCENT:
		new_state = PLOTState::TARGET_IMPACT;
		break;

	case PLOTState::TARGET_IMPACT:
		new_state = PLOTState::IDLE;
		break;

	case PLOTState::IDLE: // Fallthrough
	default:
		new_state = PLOTState::IDLE;
		break;
	}

	_plot_state = new_state;
}

void PLOT::set_plot_item()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon,
				       _global_pos_sub.get().lat, _global_pos_sub.get().lon);

	const bool is_close_to_destination = destination_dist < _param_plot_min_dist.get();

	float altitude_acceptance_radius = static_cast<float>(NAN);

	switch (_plot_state) {

	case PLOTState::MOVE_TO_TARGET: {
			PX4_INFO("PLOT State: MOVE_TO_TARGET");

			PositionYawSetpoint pos_yaw_sp {
				.lat = _crash_approach.entry_lat,
				.lon = _crash_approach.entry_lon,
				.alt = _plot_alt,
			};

			// already set final yaw if close to destination and weather vane is disabled
			pos_yaw_sp.yaw = (is_close_to_destination && !_param_wv_en.get()) ? _destination.yaw : NAN;
			setGlideToTargetMissionItem(_mission_item, pos_yaw_sp, PLOT_DESCENT_RADIUS_DEFAULT);

			break;
		}

	case PLOTState::TRANSITION_TO_DESCEND: {
			PX4_INFO("PLOT State: TRANSITION_TO_DESCEND");
			PositionYawSetpoint pos_yaw_sp{_destination};

			setTransitionToDescendMissionItem(_mission_item, pos_yaw_sp);

			break;
		}

	case PLOTState::STEEP_DESCENT: {
			PX4_INFO("PLOT State: STEEP_DESCENT");
			PositionYawSetpoint pos_yaw_sp{_destination};

			const float descent_buffer = 2.0f; // Small buffer in meters above target
			pos_yaw_sp.alt = _destination.alt + descent_buffer;

			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled

			setSteepDescentMissionItem(_mission_item, pos_yaw_sp);

			// set previous item location to loiter location such that vehicle tracks line between loiter
			// location and land location after exiting the loiter circle
			pos_sp_triplet->previous.lat = _crash_approach.entry_lat;
			pos_sp_triplet->previous.lon = _crash_approach.entry_lon;
			pos_sp_triplet->previous.alt = get_absolute_altitude_for_item(_mission_item);
			pos_sp_triplet->previous.valid = true;

			break;
		}

	case PLOTState::TARGET_IMPACT: {
			PX4_INFO("PLOT State: TARGET_IMPACT");
			PositionYawSetpoint pos_yaw_sp{_destination};

			pos_yaw_sp.yaw = !_param_wv_en.get() ? _destination.yaw : NAN; // set final yaw if weather vane is disabled
			setTargetImpactMissionItem(_mission_item, pos_yaw_sp);


			mavlink_log_info(_navigator->get_mavlink_log_pub(), "PLOT: impact on target\t");
			events::send(events::ID("plot_impact_on_target"), events::Log::Info, "PLOT: impact on target");
			break;
		}

	case PLOTState::IDLE: {
			PX4_INFO("PLOT State: IDLE");
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

PLOT::PLOTState PLOT::getActivationLandState()
{
	_land_detected_sub.update();
	return PLOTState::MOVE_TO_TARGET;
}

void PLOT::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();
	}
}

rtl_time_estimate_s PLOT::calc_rtl_time_estimate()
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

		crash_point_s crash_approach = sanitizeCrashApproach(_crash_approach);

		// const float loiter_altitude = min(crash_approach.entry_altitude_m, _plot_alt);

		// Sum up time estimate for various segments of the landing procedure
		switch (start_state_for_estimate) {

		// FALLTHROUGH
		case PLOTState::MOVE_TO_TARGET: {
				matrix::Vector2f direction{};
				get_vector_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, crash_approach.entry_lat,
							    crash_approach.entry_lon, &direction(0), &direction(1));
				float move_to_land_dist{get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon, crash_approach.entry_lat, crash_approach.entry_lon)};


				_plot_time_estimator.addDistance(move_to_land_dist, direction, 0.f);
			}

		// FALLTHROUGH
		case PLOTState::TRANSITION_TO_DESCEND:
		case PLOTState::STEEP_DESCENT:
		case PLOTState::TARGET_IMPACT:
		case PLOTState::IDLE:
			// Remaining time is 0
			break;
		}
	}

	return _plot_time_estimator.getEstimate();
}

crash_point_s PLOT::sanitizeCrashApproach(crash_point_s crash_approach) const
{
	crash_point_s sanitized_crash_approach{crash_approach};

	if (!PX4_ISFINITE(crash_approach.entry_lat) || !PX4_ISFINITE(crash_approach.entry_lon)) {
		sanitized_crash_approach.entry_lat = _destination.lat;
		sanitized_crash_approach.entry_lon = _destination.lon;
	}

	if (!PX4_ISFINITE(crash_approach.entry_altitude_m)) {
		sanitized_crash_approach.entry_altitude_m = _destination.alt + _param_plot_descend_alt.get();
	}

	if (!PX4_ISFINITE(crash_approach.dive_angle_deg)) {
		sanitized_crash_approach.dive_angle_deg = PLOT_DIVE_ANGLE_DEFAULT;
	}

	if (!PX4_ISFINITE(crash_approach.dive_airspeed_ms)) {
		sanitized_crash_approach.dive_airspeed_ms = PLOT_DIVE_SPEED_DEFAULT;
	}

	if (!PX4_ISFINITE(crash_approach.max_dive_airspeed_ms)) {
		sanitized_crash_approach.max_dive_airspeed_ms = PLOT_MAX_SPEED_DEFAULT;
	}

	if (!PX4_ISFINITE(crash_approach.throttle_setting)) {
		sanitized_crash_approach.throttle_setting = PLOT_THROTTLE_DEFAULT;
	}

	if (!PX4_ISFINITE(crash_approach.terminal_maneuver)) {
		sanitized_crash_approach.terminal_maneuver = PLOT_TERM_MANVR_DEFAULT;
	}

	if (!PX4_ISFINITE(crash_approach.entry_radius_m)) {
		sanitized_crash_approach.entry_radius_m = _param_plot_loiter_rad.get();
	}

	return sanitized_crash_approach;
}

void PLOT::publish_plot_direct_navigator_mission_item()
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
