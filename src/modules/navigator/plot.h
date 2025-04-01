/***************************************************************************
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
 * @file plot.h
 *
 * Helper class for PLOT
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include "navigator_mode.h"
#include "navigation.h"

#include "plot_direct.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_status.h>
#include <uORB/topics/rtl_time_estimate.h>

class Navigator;

class PLOT : public NavigatorMode, public ModuleParams
{
public:
	PLOT(Navigator *navigator);

	~PLOT() = default;

	enum class PlotType {
		NONE,
		PLOT_DIRECT,
	};

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void initialize() override {};

	void set_plot_alt_min(bool min) { _enforce_plot_alt = min; }

	bool isLanding();

private:

	void setPlotTypeAndDestination();

	/**
	 * @brief Publish the remaining time estimate to go to the PLOT landing point.
	 *
	 */
	void publishRemainingTimeEstimate();

	/**
	 * @brief Find PLOT destination.
	 *
	 */
	void findPlotDestination(PositionYawSetpoint &plot_position, float &plot_alt);

	/**
	 * @brief calculate return altitude from cone half angle
	 *
	 * @param[in] plot_position landing position of the plot
	 * @param[in] cone_half_angle_deg half angle of the cone [deg]
	 * @return return altitude
	 */
	float calculate_return_alt_from_cone_half_angle(const PositionYawSetpoint &plot_position,
			float cone_half_angle_deg) const;


	/**
	 * @brief Update parameters
	 *
	 */
	void parameters_update();


	hrt_abstime _destination_check_time{0};

	PlotType _plot_type{PlotType::PLOT_DIRECT};

	bool _home_has_land_approach;			///< Flag if the home position has a land approach defined
	bool _one_rally_point_has_land_approach;	///< Flag if a rally point has a land approach defined

	PlotDirect _plot_direct;

	bool _enforce_plot_alt{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RTL_TYPE>)          _param_plot_type,
		(ParamInt<px4::params::RTL_CONE_ANG>)      _param_plot_cone_half_angle_deg,
		(ParamFloat<px4::params::RTL_RETURN_ALT>)  _param_plot_return_alt,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_plot_min_dist,
		(ParamFloat<px4::params::NAV_ACC_RAD>)     _param_nav_acc_rad,
		(ParamInt<px4::params::RTL_APPR_FORCE>)    _param_plot_approach_force
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};

	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
	uORB::PublicationData<rtl_status_s> _plot_status_pub{ORB_ID(rtl_status)};
};
