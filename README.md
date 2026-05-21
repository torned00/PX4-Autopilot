# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>


## Updating the Gazebo Submodule (The Falcon Project)

To use worlds and models that has been created or updated and added to the [PX4-Gazebo-models repo](https://github.com/ingeborgaarnes/PX4-gazebo-models/tree/main), you need update the Gazebo Simulation Submodule. This is simply done by running the `update_gz_submodule.sh` script that is added to this repo.



# FALCON Flight Mode

FALCON is a custom PX4 flight mode developed for precision-guided airdrops aimed at early-stage wildfire suppression. The flight mode is designed for fixed-wing gliders released from a carrier UAV and enables autonomous guidance toward a designated ground target without propulsion.

The guidance logic is divided into three phases: **Glide**, **Dive**, and **Impact**, allowing the aircraft to transition from efficient long-range flight to aggressive terminal guidance.

## Glide Phase

During the Glide phase, the aircraft flies toward the target using conventional fixed-wing guidance. Total Energy Control System (TECS) is used to regulate the aircraft energy state and maintain the desired airspeed by balancing kinetic and potential energy through pitch control. Lateral navigation is initially handled by the NPFG (Nonlinear Path Following Guidance) controller, which guides the aircraft toward the target while maintaining stable and energy-efficient flight.

The Glide phase is designed to maximize range and maintain controllability while gradually positioning the aircraft for terminal descent.

## Dive Phase

Once the aircraft reaches a predefined distance or geometry relative to the target, FALCON transitions into the Dive phase. In this phase, the guidance strategy changes from path following to direct target interception.

A proportional navigation (PROPNAV / PN) controller is used to generate aggressive terminal guidance commands based on the line-of-sight (LOS) dynamics between the aircraft and the target. The controller continuously adjusts the aircraft trajectory to minimize miss distance and steer the aircraft onto a collision course.

 HOME POSITION IS USED AS THE TARGET LOCATION!

The Dive phase increases descent angle and targeting precision while reducing the effect of accumulated navigation errors from the glide segment.

## Impact Phase

As the aircraft approaches the target and the estimated time-to-go becomes sufficiently small, FALCON transitions into the Impact phase. During this phase, the commanded pitch and roll is held approximately constant to stabilize the final descent trajectory and reduce excessive manoeuvring close to impact.

The objective of the Impact phase is to maintain a predictable terminal trajectory and ensure accurate payload delivery onto the target location.

## Motivation

The flight mode was developed as part of a master thesis investigating low-cost autonomous wildfire suppression systems using guided airdrops. The primary goal is to achieve meter-level targeting accuracy while operating under realistic environmental disturbances such as wind and gusts.

Unlike traditional waypoint navigation, FALCON is specifically designed for terminal interception problems, where minimizing miss distance is more important than smooth path following.

## Current Status

The system has been validated through:
- Software-in-the-loop (SITL) simulation, achieved sub-meter level accuracy.
- Real-world flight testing
- Comparison between simulated and measured trajectories

The current implementation demonstrates successful autonomous transitions between all guidance phases and significantly improved terminal accuracy using PN-based lateral guidance compared to the original NPFG-based implementation.

Future work includes:
- Improved state estimation and wind compensation
- Structural refinement of the prototype airframe
- Higher-fidelity aerodynamic modelling
- Full payload integration and release testing

## Params

Initiate FALCON flight mode in terminal: commander mode auto:falcon
Initiate FALCON flight mode through RC control or QGC by switching to "Precision Landing" mode.

FALCON params:
FALCON_DIVE, transition criteria between Glide and Dive.
PROPNAV_N, Proportional navigation controller gain.
PROPNAV_TTG, used to transition from Dive over to Impact.
PROPNAV_OFFSET, moves the target "offset altitude" above ground, used for practical testing of the flight mode.
