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
 * @file plot_direct.h
 *
 * Helper class for PLOT mode
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/Vector2.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

#include <lib/rtl/rtl_time_estimator.h>
#include "mission_block.h"
#include "navigation.h"
#include "crash_land.hpp"

using namespace time_literals;

class Navigator;

class PlotDirect : public MissionBlock, public ModuleParams
{
public:
	PlotDirect(Navigator *navigator);

	~PlotDirect() = default;

	/**
	 * @brief on inactivation
	 *
	 */
	void on_inactivation() override;

	/**
	 * @brief on activation.
	 * Initialize the return to launch calculations.
	 *
	 */
	void on_activation() override;

	/**
	 * @brief on active
	 * Update the return to launch calculation and set new setpoints for controller if necessary.
	 *
	 */
	void on_active() override;

	/**
	 * @brief on inactive
	 * Poll required topics also when incative for plot time estimate.
	 *
	 */
	void on_inactive() override;

	/**
	 * @brief Calculate the estimated time needed to perform precision landing.
	 *
	 * @return estimated time to perform precision landing, based on rtl time estimate.
	 */
	rtl_time_estimate_s calc_rtl_time_estimate();

	void setPlotAltMin(bool min) { _enforce_plot_alt = min; }
	void setPlotAlt(float alt) {_plot_alt = alt;};

	void setPlotPosition(PositionYawSetpoint position, crash_point_s crash_pos);

	bool isLanding() { return (_plot_state == PLOTState::HIT_TARGET || _plot_state == PLOTState::MOVE_TO_LAND);};

private:
	/**
	 * @brief Return to launch state machine.
	 *
	 */
	enum class PLOTState {
		MOVE_TO_TARGET,
		MOVE_TO_LAND,
		HIT_TARGET,
		IDLE
	} _plot_state{PLOTState::IDLE}; /*< Current state in the state machine.*/

private:
	/**
	 * @brief Update the PLOT state machine.
	 *
	 */
	void _updatePlotState();

	/**
	 * @brief Set the return to launch control setpoint.
	 *
	 */
	void set_plot_item();

	/**
	 * @brief sanitize crash_approach
	 *
	 */
	crash_point_s sanitizeCrashApproach(crash_point_s crash_approach) const;

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	/**
	 * @brief Publish navigator mission item
	 *
	 */
	void publish_plot_direct_navigator_mission_item();

	PLOTState getActivationLandState();

	void setLoiterPosition(); // Do not need this?

	bool _enforce_plot_alt{false};
	bool _force_heading{false};
	RtlTimeEstimator _plot_time_estimator;

	PositionYawSetpoint _destination; ///< the PLOT position to fly to
	crash_point_s _crash_approach;

	float _plot_alt{0.0f};	///< AMSL altitude at which the vehicle should perform precision landing

	static constexpr float PLOT_DIVE_ANGLE_DEFAULT = -45.0f;     // degrees
	static constexpr float PLOT_DIVE_SPEED_DEFAULT = 20.0f;      // m/s
	static constexpr float PLOT_MAX_SPEED_DEFAULT = 30.0f;       // m/s
	static constexpr float PLOT_THROTTLE_DEFAULT = 0.0f;         // normalized (0-1)
	static constexpr int PLOT_TERM_MANVR_DEFAULT = 0;            // 0=none, 1=pitch down, 2=roll

	DEFINE_PARAMETERS( // Do not need these?
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_plot_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>)  _param_plot_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_plot_min_dist,
		(ParamInt<px4::params::RTL_PLD_MD>)        _param_plot_pld_md,
		(ParamFloat<px4::params::RTL_LOITER_RAD>)  _param_plot_loiter_rad,

		// external params
		(ParamBool<px4::params::WV_EN>) _param_wv_en
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};		/**< home position subscription */
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};	/**< vehicle status subscription */
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};
	uORB::Publication<navigator_mission_item_s> _navigator_mission_item_pub{ORB_ID::navigator_mission_item}; /**< Navigator mission item publication*/
};
