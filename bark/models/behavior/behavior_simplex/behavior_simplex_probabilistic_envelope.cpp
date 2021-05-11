// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>
#include <optional>
#include <tuple>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/behavior/behavior_simplex/behavior_simplex_probabilistic_envelope.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/commons/distribution/multivariate_normal.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::observer::ObserverModelParametric;
using bark::commons::MultivariateDistribution;


Eigen::MatrixXd GetObserverCovariance(const ObservedWord& observed_world) {
  const auto& observer_parametric = std::dynamic_pointer_cast<ObserverModelParametric>(
                                                        observed_world.GetObserverModel());
  BARK_EXPECT_TRUE(bool(observer_parametric));
  const auto& multivariate_dist = std::dynamic_pointer_cast<MultivariateDistribution>(
                  observer_parametric->GetOthersStateDeviationDist());
  BARK_EXPECT_TRUE(bool(multivariate_dist));
  return multivariate_dist->GetCovariance();
}

std::pair<EnvelopeProbabilityList, ViolationProbabilityList> 
                                          CalculateAgentsWorstCaseEnvelopes(const AgentPtr& ego_agent,
                                                                            const AgentPtr& other_agent, 
                                                                            const std::vector<double> iso_discretizations,
                                                                            const Eigen::MatrixXd& observer_covariance) {
  
}

EnvelopeProbabilityPair CalculateProbabilisticEnvelope(const EnvelopeProbabilityList& envelope_probability_list, const Probability& violation_threshold); 

Probability CalculateExpectedViolation(const ViolationProbabilityList& violation_probability_list);





Trajectory BehaviorSimplexProbabilisticEnvelope::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  // Collect Worst-Case Envelopes over all other agents
  const auto ego_agent = observed_world.GetEgoAgent();
  const auto observer_covariance = GetObserverCovariance(observed_world);
  EnvelopeProbabilityList envelopes;
  ViolationProbabilityList violations;
  for (const auto& other_agent : observed_world.GetOtherAgents()) {
    envelopes_violations = CalculateAgentsWorstCaseEnvelopes(ego_agent, other_agent, iso_probability_discretizations_, observer_covariance);
    const auto& agent_envelopes = std::get<0>(envelopes_violations);
    const auto& agent_violations = std::get<1>(envelopes_violations);
    MoveAppend(agent_envelopes, envelopes);
    MoveAppend(agent_violations, violations);
  }

  // Calculate probablistic envelope and expected violation
  EnvelopeProbabilityPair probabilistic_envelope_pair =
           CalculateProbabilisticEnvelope(envelopes, violation_threshold_); 
  current_expected_safety_violation_ = CalculateExpectedViolation(agent_violations);
  
  // Handling switching condition and envelope restriction 
  Action last_action;
  dynamic::Trajectory last_traj;
  if (current_expected_safety_violation_ > violation_threshold_) {
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
    behavior_rss_status_ = BehaviorRSSConformantStatus::SAFETY_BEHAVIOR;
  } else {
    nominal_behavior_model_->Plan(min_planning_time, observed_world);
    last_action = nominal_behavior_model_->GetLastAction();
    last_traj = nominal_behavior_model_->GetLastTrajectory();
    behavior_rss_status_ = BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR;
  }
  SetLastTrajectory(last_traj);
  SetLastAction(last_action);
  return last_traj;
}


}  // namespace behavior
}  // namespace models
}  // namespace bark
