// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/rule_based/rule_based.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <tuple>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/dynamic/single_track.hpp"
#include "modules/models/dynamic/integration.hpp"


namespace modules {
namespace models {
namespace behavior {

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Point2d;
using modules::models::dynamic::State;
using modules::models::dynamic::StateDefinition;
using modules::world::objects::Agent;
using modules::world::objects::AgentPtr;
using modules::models::dynamic::DynamicModelPtr;

Trajectory BehaviorRuleBased::Plan(
    float delta_time, const world::ObservedWorld& observed_world) {
  using dynamic::StateDefinition;
  SetBehaviorStatus(BehaviorStatus::VALID);

  if (!observed_world.GetLaneCorridor()) {
    return GetLastTrajectory();
  }

  // whether to change lanes or not
  std::pair<LaneChangeDecision, LaneCorridorPtr> lane_res =
    CheckIfLaneChangeBeneficial(observed_world);
  SetLaneCorridor(lane_res.second);

  // we want to calc. the acc. based on the actual LaneCorridor
  std::tuple<double, double, bool> rel_values = CalcRelativeValues(
    observed_world,
    GetLaneCorridor());

  std::tuple<Trajectory, Action> traj_action =
    GenerateTrajectory(
      observed_world, GetLaneCorridor(), rel_values, delta_time);

  // set values
  Trajectory traj = std::get<0>(traj_action);
  Action action = std::get<1>(traj_action);
  SetLastTrajectory(traj);
  SetLastAction(action);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
