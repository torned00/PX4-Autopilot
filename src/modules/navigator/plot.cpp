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

PLOT::PLOT(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PLOT),
	ModuleParams(navigator)
{
	_destination.lat = static_cast<double>(NAN);
	_destination.lon = static_cast<double>(NAN);

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
		initializePlotDestination();
	}

}

void PLOT::on_activation()
{
	initializePlotDestination();

	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_land_detected_sub.update();

	parameters_update();

	_plot_state = PLOTState::MOVE_TO_TARGET;

	// reset cruising speed and throttle to default for PLOT
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

	set_plot_navigator_mission_item();

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
		set_plot_navigator_mission_item();
	}


}

void PLOT::initializePlotDestination()
{
	if (isActive()) {
		// Already in motion, no changes allowed
		return;
	}

	const home_position_s &home = _home_pos_sub.get();

	// Build destination directly from home
	_destination.lat = PX4_ISFINITE(home.lat) ? home.lat : 0.0;
	_destination.lon = PX4_ISFINITE(home.lon) ? home.lon : 0.0;
	_destination.alt = PX4_ISFINITE(home.alt) ? home.alt : 0.0f;
	_destination.yaw = PX4_ISFINITE(home.yaw) ? home.yaw : NAN;

	// Optional: fallback sanity check
	if (!PX4_ISFINITE(_destination.lat) || !PX4_ISFINITE(_destination.lon)) {
		PX4_WARN("PLOT: Home position invalid, cannot set destination");
	}

	_plot_alt = _global_pos_sub.get().alt;

	_force_heading = false;
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

void PLOT::set_plot_navigator_mission_item()
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
				.lat = _destination.lat,
				.lon = _destination.lon,
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

	publish_plot_navigator_mission_item(); // for logging
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

void PLOT::publish_plot_navigator_mission_item()
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
	navigator_mission_item.frame = _mission_item.origin; // Is this a double assignment?

	navigator_mission_item.loiter_exit_xtrack = _mission_item.loiter_exit_xtrack;
	navigator_mission_item.force_heading = _mission_item.force_heading;
	navigator_mission_item.altitude_is_relative = _mission_item.altitude_is_relative;
	navigator_mission_item.autocontinue = _mission_item.autocontinue;
	navigator_mission_item.vtol_back_transition = _mission_item.vtol_back_transition;

	navigator_mission_item.timestamp = hrt_absolute_time();

	_navigator_mission_item_pub.publish(navigator_mission_item);
}
