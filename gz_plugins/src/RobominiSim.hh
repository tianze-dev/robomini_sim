/*
 * RobominiSim.hh — world-level Gazebo system plugin for the ROBOMINI
 * "机甲杯" (Robomini) auto-aim simulation platform.
 *
 * The plugin implements the Gazebo-side interface contract:
 *
 *   rune (energy mechanism):
 *     sub  /rune/<team>/mode    {"mode":"fixed|random_fixed|small|big","a":..,"omega":..,"t0":..}
 *     sub  /rune/<team>/lights  {"targets":["off|pending|active" x5]}
 *     pub  /rune/<team>/state   20 Hz {"hub_angle","speed","mode","targets","valid"}
 *     pub  /rune/<team>/hit     {"target","ball","speed","direct","pos","stamp"}
 *
 *   launcher:
 *     sub  /launcher/<team>/gimbal {"yaw":..,"pitch":..}   (joint angles, rad)
 *     sub  /launcher/<team>/fire   {"speed":..}            (m/s, default 12)
 *     pub  /launcher/<team>/state  20 Hz {"yaw","pitch","muzzle","balls_fired","ready"}
 *
 * All topics are gz.msgs.StringMsg carrying UTF-8 JSON, as required by the
 * contract. No ROS dependency: everything is gz-transport + ECM components.
 *
 * SDF usage (attached to the <world>):
 *
 *   <plugin filename="librobomini_sim_system.so"
 *           name="robomini::sim::RobominiSim">
 *     <world_name>robomini</world_name>
 *     <publish_rate>20</publish_rate>
 *     ...
 *   </plugin>
 *
 * See README.md for the full parameter list, build and test instructions.
 *
 * License: Apache-2.0 (matches gz-sim). Author: gz-plugins teammate.
 */
#ifndef ROBOMINI_SIM_SYSTEM_HH_
#define ROBOMINI_SIM_SYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace robomini
{
namespace sim
{

/// \brief World system plugin that drives the energy rune and the two
/// launchers of the Robomini simulation.
class RobominiSim : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
  /// \brief Constructor
  public: RobominiSim();

  /// \brief Destructor
  public: ~RobominiSim() override;

  // Documentation inherited.
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override;

  // Documentation inherited.
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;

  /// \brief PIMPL so the header stays stable / free of transport internals.
  private: class Impl;
  private: std::unique_ptr<Impl> dataPtr;
};

}  // namespace sim
}  // namespace robomini

#endif  // ROBOMINI_SIM_SYSTEM_HH_
