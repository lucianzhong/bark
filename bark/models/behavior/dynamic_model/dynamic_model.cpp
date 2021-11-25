// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include <cmath>
#include "bark/models/dynamic/integration.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Input;
using dynamic::SingleTrackModel;
using dynamic::StateDefinition;
using dynamic::StateDefinition::THETA_POSITION;
using dynamic::StateDefinition::TIME_POSITION;
using dynamic::StateDefinition::VEL_POSITION;
using dynamic::StateDefinition::X_POSITION;
using dynamic::StateDefinition::Y_POSITION;

BehaviorDynamicModel::BehaviorDynamicModel(const commons::ParamsPtr& params)
    : BehaviorModel(params),
      integration_time_delta_(
          params->GetReal("BehaviorDynamicModel::IntegrationTimeDelta",
                          "delta t for integration", 0.05)),
         kappa_max_(params->GetReal("BehaviorDynamicModel::KappaMax",
                          "max steering rate", 0.03)) {
      Input inp(2);
      inp << 0., 0.;
      action_ = inp;
  }

dynamic::Trajectory BehaviorDynamicModel::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);
  const DynamicModelPtr dynamic_model =
      observed_world.GetEgoAgent()->GetDynamicModel();
  auto single_track =
      std::dynamic_pointer_cast<SingleTrackModel>(dynamic_model);
  if (!single_track)
    LOG(FATAL) << "Only SingleTrack as dynamic model supported!";

  dynamic::State ego_vehicle_state =
      observed_world.GetEgoAgent()->GetCurrentState();
  Input last_input = boost::get<Input>(action_);

  LOG(INFO) << "hist size = " << observed_world.GetEgoAgent()->GetStateInputHistory().size();
  try {
      Input last_input = boost::get<Input>(
      observed_world.GetEgoAgent()->GetLastAction());
  } catch(boost::bad_get) {}
  
  double start_time = observed_world.GetWorldTime();
  double dt = integration_time_delta_;
  int num_trajectory_points =
      static_cast<int>(std::ceil(min_planning_time / dt)) + 1;

  dynamic::Trajectory traj(num_trajectory_points,
                           static_cast<int>(StateDefinition::MIN_STATE_SIZE));

  // this action is set externally. e.g. by RL
  Input current_input = boost::get<Input>(action_);

  // Handle steering limits when second input is kappa
  const double last_delta = last_input(1);
  const double current_delta = current_input(1);
  LOG(INFO) << "current input: " << current_input(1) << ", last input: " << last_input(1);
  if( (current_delta - last_delta) / min_planning_time > kappa_max_) {
    current_input(1) = current_delta + min_planning_time * kappa_max_;
    LOG(INFO) << "limiting c=" << current_delta << " and l=" << last_delta <<  " to " << current_input(1);
  } else if ( (current_delta - last_delta) / min_planning_time < - kappa_max_) {
    current_input(1) = current_delta - min_planning_time * kappa_max_;
    LOG(INFO) << "limiting c=" << current_delta << " and l=" << last_delta <<  " to " << current_input(1);
  }

  // generate a trajectory
  traj.row(0) = ego_vehicle_state;
  for (int i = 1; i < num_trajectory_points; i++) {
    auto next_state =
        dynamic::euler_int(*dynamic_model, traj.row(i - 1), current_input, dt);
    traj.row(i) = next_state;
    traj(i, 0) = start_time + i * dt;
  }

  this->SetLastTrajectory(traj);
  this->SetLastAction(current_input);
  return traj;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark
