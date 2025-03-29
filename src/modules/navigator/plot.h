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
#include <dataman_client/DatamanClient.hpp>
#include "rtl_base.h"
#include "plot_direct.h"
#include "rtl_direct_mission_land.h"
#include "rtl_mission_fast.h"
#include "rtl_mission_fast_reverse.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
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
		PLOT_DIRECT_MISSION_LAND,
		PLOT_MISSION_FAST,
		PLOT_MISSION_FAST_REVERSE,
	};

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void initialize() override {};

	void set_plot_alt_min(bool min) { _enforce_plot_alt = min; }

	void updateSafePoints(uint32_t new_safe_point_id) { _initiate_safe_points_updated = true; _safe_points_id = new_safe_point_id; }

	bool isLanding();

private:
	enum class DestinationType {
		DESTINATION_TYPE_HOME,
		DESTINATION_TYPE_MISSION_LAND,
		DESTINATION_TYPE_SAFE_POINT,
	};

private:

	/**
	 * @brief Check mission landing validity
	 * @return true if mission has a land start, a land and is valid
	 */
	bool hasMissionLandStart() const;

	/**
	 * @brief function to call regularly to do background work
	 */
	void updateDatamanCache();

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
	void findPlotDestination(DestinationType &destination_type, PositionYawSetpoint &plot_position, float &plot_alt,
				uint8_t &safe_point_index);

	/**
	 * @brief Set the position of the land start marker in the planned mission as destination.
	 *
	 */
	void setLandPosAsDestination(PositionYawSetpoint &plot_position, mission_item_s &land_mission_item) const;

	/**
	 * @brief Set the safepoint as destination.
	 *
	 * @param mission_safe_point is the mission safe point/rally point to set as destination.
	 */
	void setSafepointAsDestination(PositionYawSetpoint &plot_position, const mission_item_s &mission_safe_point) const;

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
	 * @brief initialize PLOT mission type
	 *
	 */
	void init_plot_mission_type();

	/**
	 * @brief Update parameters
	 *
	 */
	void parameters_update();

	/**
	 * @brief read VTOL land approaches
	 *
	 * @param[in] plot_position landing position of the plot
	 *
	 */
	land_approaches_s readVtolLandApproaches(PositionYawSetpoint plot_position) const;

	/**
	 * @brief Has VTOL land approach
	 *
	 * @param[in] plot_position landing position of the plot
	 *
	 * @return true if home land approaches are defined for home position
	 * @return false otherwise
	 */
	bool hasVtolLandApproach(const PositionYawSetpoint &plot_position) const;

	/**
	 * @brief Choose best landing approach
	 *
	 * Choose best landing approach for home considering wind
	 *
	 * @return loiter_point_s best landing approach
	 */
	loiter_point_s chooseBestLandingApproach(const land_approaches_s &vtol_land_approaches);

	enum class DatamanState {
		UpdateRequestWait,
		Read,
		ReadWait,
		Load,
		Error
	};

	hrt_abstime _destination_check_time{0};

	RtlBase *_plot_mission_type_handle{nullptr};
	PlotType _set_plot_mission_type{PlotType::NONE};

	PlotType _plot_type{PlotType::PLOT_DIRECT};

	bool _home_has_land_approach;			///< Flag if the home position has a land approach defined
	bool _one_rally_point_has_land_approach;	///< Flag if a rally point has a land approach defined

	DatamanState _dataman_state{DatamanState::UpdateRequestWait};
	DatamanState _error_state{DatamanState::UpdateRequestWait};
	uint32_t _opaque_id{0}; ///< dataman safepoint id: if it does not match, safe points data was updated
	bool _safe_points_updated{false}; ///< flag indicating if safe points are updated to dataman cache
	mutable DatamanCache _dataman_cache_safepoint{"plot_dm_cache_miss_geo", 4};
	DatamanClient	&_dataman_client_safepoint = _dataman_cache_safepoint.client();
	bool _initiate_safe_points_updated{true}; ///< flag indicating if safe points update is needed
	mutable DatamanCache _dataman_cache_landItem{"plot_dm_cache_miss_land", 2};
	uint32_t _mission_id = 0u;
	uint32_t _safe_points_id = 0u;

	mission_stats_entry_s _stats;

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
	uORB::SubscriptionData<mission_s> _mission_sub{ORB_ID(mission)};
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};
	uORB::SubscriptionData<wind_s>		_wind_sub{ORB_ID(wind)};

	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
	uORB::PublicationData<rtl_status_s> _plot_status_pub{ORB_ID(rtl_status)};
};
