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
 * @file falcon.h
 *
 * Class for FALCON
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ingeborg Aarnes <ingebota@stud.ntnu.no>
 */

#pragma once

#include <px4_platform_common/module_params.h>

#include "mission_block.h"
#include "navigator_mode.h"
#include "navigation.h"

#include <matrix/Vector2.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/home_position.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

using namespace time_literals;

class Navigator;

class FALCON : public MissionBlock, public ModuleParams
{
public:
	FALCON(Navigator *navigator);

	~FALCON() = default;
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
	 * Poll required topics also when incative for falcon time estimate.
	 *
	 */
	void on_inactive() override;

	void initialize() override {};

	void initializeFalconDestination();

	bool isLanding() { return (_falcon_state == FALCONState::IMPACT || _falcon_state == FALCONState::DIVE);};

private:
	/**
	 * @brief Return to launch state machine.
	 *
	 */
	enum class FALCONState {
		GLIDE,
		PITCH,
		DIVE,
		IMPACT
	} _falcon_state{FALCONState::GLIDE}; /*< Current state in the state machine.*/

	/**
	 * @brief Update the FALCON state machine.
	 *
	 */
	void _updateFalconState();

	/**
	 * @brief Set the return to launch control setpoint.
	 *
	 */
	void set_falcon_navigator_mission_item(); // Consider renaming to make more sense

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	/**
	 * @brief Publish navigator mission item
	 *
	 */
	void publish_falcon_navigator_mission_item();

	hrt_abstime _destination_check_time{0};

	bool _force_heading{false};

	PositionYawSetpoint _destination; ///< the FALCON position to fly to

	float _falcon_alt{0.0f};	///< AMSL altitude at which the vehicle should perform precision landing

	static constexpr float FALCON_DESCENT_RADIUS_DEFAULT = 140.0f;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_MIN_DIST>)    _param_falcon_min_dist,
		// external params
		(ParamBool<px4::params::WV_EN>) 	_param_wv_en
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<vehicle_status_s> _vehicle_status_sub{ORB_ID(vehicle_status)};			/**< vehicle status subscription */
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};				/**< home position subscription */
	uORB::SubscriptionData<vehicle_land_detected_s> _land_detected_sub{ORB_ID(vehicle_land_detected)};	/**< vehicle land detected subscription */
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};

	uORB::Publication<navigator_mission_item_s> _navigator_mission_item_pub{ORB_ID::navigator_mission_item}; /**< Navigator mission item publication*/
};
