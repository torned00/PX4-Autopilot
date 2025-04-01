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
 * Helper class to access PLOT
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#include "plot.h"
#include "navigator.h"
#include "mission_block.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>

using namespace time_literals;
using namespace math;
using matrix::wrap_pi;

static constexpr float MIN_DIST_THRESHOLD = 2.f;

PLOT::PLOT(Navigator *navigator) :
	NavigatorMode(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PLOT),
	ModuleParams(navigator),
	_plot_direct(navigator)
{
	_plot_direct.initialize();
}

void PLOT::on_inactive()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();


	parameters_update();


	_plot_direct.run(false);

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
		switch (_plot_type) {
		case PlotType::PLOT_DIRECT:
			estimated_time = _plot_direct.calc_rtl_time_estimate();
			break;

		default:
			break;
		}
	}

	_rtl_time_estimate_pub.publish(estimated_time);
}

void PLOT::on_activation()
{
	setPlotTypeAndDestination();

	switch (_plot_type) {

	case PlotType::PLOT_DIRECT:
		_plot_direct.setPlotAltMin(_enforce_plot_alt);
		break;

	default:
		break;
	}

	// set gimbal to neutral position (level with horizon) to reduce change of damage on landing
	_navigator->acquire_gimbal_control();
	_navigator->set_gimbal_neutral();
	_navigator->release_gimbal_control();
}

void PLOT::on_active()
{
	_global_pos_sub.update();
	_vehicle_status_sub.update();
	_home_pos_sub.update();
	_wind_sub.update();

	switch (_plot_type) {
	case PlotType::PLOT_DIRECT:
		_plot_direct.run(true);
		break;

	default:
		break;
	}

	// Keep publishing remaining time estimates every 2 seconds
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 2_s) {
		_destination_check_time = now;
		publishRemainingTimeEstimate();
	}
}

bool PLOT::isLanding()
{
	bool is_landing{false};

	switch (_plot_type) {

	case PlotType::PLOT_DIRECT:
		is_landing = _plot_direct.isLanding();
		break;

	default:
		break;
	}

	return is_landing;
}

void PLOT::setPlotTypeAndDestination()
{

	if (_param_plot_type.get() != 2) {
		// check the closest allowed destination.
		PositionYawSetpoint plot_position;
		float plot_alt;
		findPlotDestination(plot_position, plot_alt);


		loiter_point_s landing_loiter;
		landing_loiter.lat = plot_position.lat;
		landing_loiter.lon = plot_position.lon;
		landing_loiter.height_m = NAN;


		_plot_type = PlotType::PLOT_DIRECT;
		_plot_direct.setPlotAlt(plot_alt);
		_plot_direct.setPlotPosition(plot_position, landing_loiter);

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

	if (_param_plot_cone_half_angle_deg.get() > 0
	    && _vehicle_status_sub.get().vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		plot_alt = calculate_return_alt_from_cone_half_angle(plot_position, (float)_param_plot_cone_half_angle_deg.get());

	} else {
		plot_alt = max(_global_pos_sub.get().alt, plot_position.alt + _param_plot_return_alt.get());
	}
}


float PLOT::calculate_return_alt_from_cone_half_angle(const PositionYawSetpoint &plot_position,
		float cone_half_angle_deg) const
{
	// horizontal distance to destination
	const float destination_dist = get_distance_to_next_waypoint(_global_pos_sub.get().lat, _global_pos_sub.get().lon,
				       plot_position.lat, plot_position.lon);

	// minium plot altitude to use when outside of horizontal acceptance radius of target position.
	// We choose the minimum height to be two times the distance from the land position in order to
	// avoid the vehicle touching the ground while still moving horizontally.
	const float return_altitude_min_outside_acceptance_rad_amsl = plot_position.alt + 2.0f * _param_nav_acc_rad.get();

	const float max_return_altitude = plot_position.alt + _param_plot_return_alt.get();

	float return_altitude_amsl = max_return_altitude;

	if (destination_dist <= _param_nav_acc_rad.get()) {
		return_altitude_amsl = plot_position.alt + 2.0f * destination_dist;

	} else {
		if (destination_dist <= _param_plot_min_dist.get()) {

			// constrain cone half angle to meaningful values. All other cases are already handled above.
			const float cone_half_angle_rad = radians(constrain(cone_half_angle_deg, 1.0f, 89.0f));

			// minimum altitude we need in order to be within the user defined cone
			const float cone_intersection_altitude_amsl = destination_dist / tanf(cone_half_angle_rad) + plot_position.alt;

			return_altitude_amsl = min(cone_intersection_altitude_amsl, return_altitude_amsl);
		}

		return_altitude_amsl = max(return_altitude_amsl, return_altitude_min_outside_acceptance_rad_amsl);
	}

	return constrain(return_altitude_amsl, _global_pos_sub.get().alt, max_return_altitude);
}


void PLOT::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

		if (!isActive()) {
			setPlotTypeAndDestination();
		}
	}
}

