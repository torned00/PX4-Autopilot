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


		crash_point_s landing_pos;
		landing_pos.entry_lat = plot_position.lat;
		landing_pos.entry_lon = plot_position.lon;
		landing_pos.entry_altitude_m = plot_alt;


		_plot_type = PlotType::PLOT_DIRECT;
		_plot_direct.setPlotAlt(plot_alt);
		_plot_direct.setPlotPosition(plot_position, landing_pos);

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

	plot_alt = max(_global_pos_sub.get().alt, plot_position.alt + _param_plot_return_alt.get());
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

