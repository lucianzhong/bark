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
#include <vector>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/behavior/behavior_simplex/behavior_simplex_probabilistic_envelope.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/commons/distribution/multivariate_normal.hpp"
#include "bark/models/observer/utils/ellipsis.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::observer::ObserverModelParametric;
using bark::commons::MultivariateDistribution;
using bark::models::observer::utils::ObserveAtIsoLine;
using bark::models::dynamic::AccelerationLimits;
using world::ObservedWorld;
using world::World;
using dynamic::Trajectory;
using bark::geometry::Polygon;
using world::evaluation::BaseEvaluator;
using world::objects::AgentId;
using world::objects::AgentPtr;
using bark::commons::Probability;
using bark::models::dynamic::State;
using bark::models::dynamic::StateDefinition::X_POSITION;
using bark::models::dynamic::StateDefinition::Y_POSITION;
using bark::geometry::Point2d;

Eigen::MatrixXd GetObserverCovariance(const ObservedWorld& observed_world) {
  const auto& observer_parametric = std::dynamic_pointer_cast<ObserverModelParametric>(
                                                        observed_world.GetObserverModel());
  BARK_EXPECT_TRUE(bool(observer_parametric));
  const auto& multivariate_dist = std::dynamic_pointer_cast<MultivariateDistribution>(
                  observer_parametric->GetOthersStateDeviationDist());
  BARK_EXPECT_TRUE(bool(multivariate_dist));
  return multivariate_dist->GetCovariance();
}

template<typename F>
void SortEnvelopes(
  EnvelopeProbabilityList& envelope_probability_list,
  F& lambda) {
  std::sort(
    envelope_probability_list.begin(),
    envelope_probability_list.end(), lambda);
}

std::vector<EnvelopeProbabilityList> MinMaxEnvelopeValues(
  EnvelopeProbabilityList& envelope_probability_list) {

  EnvelopeProbabilityList lat_min_prob_list = envelope_probability_list;
  EnvelopeProbabilityList lat_max_prob_list = envelope_probability_list;
  EnvelopeProbabilityList lon_min_prob_list = envelope_probability_list;
  EnvelopeProbabilityList lon_max_prob_list = envelope_probability_list;

  // this needs to be sorted in four directions
  auto lambda_lat_min = [](
    const EnvelopeProbabilityPair ep1, const EnvelopeProbabilityPair& ep2){
    return ep1.first.lat_acc_min < ep2.first.lat_acc_min;
  };
  auto lambda_lat_max = [](
    const EnvelopeProbabilityPair ep1, const EnvelopeProbabilityPair& ep2){
    return ep1.first.lat_acc_max > ep2.first.lat_acc_max;
  };
  auto lambda_lon_min = [](
    const EnvelopeProbabilityPair ep1, const EnvelopeProbabilityPair& ep2){
    return ep1.first.lon_acc_min < ep2.first.lon_acc_min;
  };
  auto lambda_lon_max = [](
    const EnvelopeProbabilityPair ep1, const EnvelopeProbabilityPair& ep2){
    return ep1.first.lon_acc_max > ep2.first.lon_acc_max;
  };

  std::vector<EnvelopeProbabilityList> env_prob_list;
  SortEnvelopes(lat_min_prob_list, lambda_lat_min);
  SortEnvelopes(lat_max_prob_list, lambda_lat_min);
  SortEnvelopes(lon_min_prob_list, lambda_lon_min);
  SortEnvelopes(lon_max_prob_list, lambda_lon_min);
  env_prob_list.push_back(lat_min_prob_list);
  env_prob_list.push_back(lat_max_prob_list);
  env_prob_list.push_back(lon_min_prob_list);
  env_prob_list.push_back(lon_max_prob_list);

  return env_prob_list;
}

std::pair<EnvelopeProbabilityList, ViolationProbabilityList>
                                          CalculateAgentsWorstCaseEnvelopes(const ObservedWorld& ego_only_world,
                                                                            const AgentPtr& other_agent,
                                                                            const std::vector<double> iso_discretizations,
                                                                            const Eigen::MatrixXd& observer_covariance,
                                                                            const std::vector<double> angular_discretization,
                                                                            std::shared_ptr<BaseEvaluator>& evaluator,
                                                                            const double min_planning_time) {
  EnvelopeProbabilityList agent_envelopes;
  ViolationProbabilityList agent_violations;
  for (std::size_t iso_prob_idx = 0; iso_prob_idx < iso_discretizations.size(); ++iso_prob_idx) {
    // Create agent variations at a certain iso line
    const auto& other_agent_variations = ObserveAtIsoLine(other_agent, angular_discretization,
                                                        observer_covariance, iso_discretizations.at(iso_prob_idx));
    EnvelopeProbabilityList agent_iso_envelopes;
    bool agent_violates_at_iso = false;
    // Create all envelopes for these agent variations
    for (const auto&  other_varied_agent : other_agent_variations) {
      bool agent_violated;
      Envelope agent_envelope;
      std::tie(agent_violated, agent_envelope) = GetViolatedAndEnvelope(ego_only_world, other_varied_agent, evaluator, min_planning_time);
      agent_iso_envelopes.push_back(EnvelopeProbabilityPair(agent_envelope, iso_prob_idx));
      agent_violates_at_iso = agent_violates_at_iso || agent_violated;
    }

    // sort based on min, max lat and long
    auto envelopes = MinMaxEnvelopeValues(agent_iso_envelopes);
    auto env_prob = EnvelopeProbabilityPair(
      Envelope{
        envelopes[0][0].first.lat_acc_min,
        envelopes[1][0].first.lat_acc_max,
        envelopes[2][0].first.lon_acc_min,
        envelopes[3][0].first.lon_acc_max}, iso_prob_idx);
    agent_envelopes.push_back(env_prob);

    if(agent_violates_at_iso) {
      agent_violations.push_back(ViolationProbabilityPair(true, iso_prob_idx));
    }
  }
  return std::make_pair(agent_envelopes, agent_violations);
}

std::pair<bool, Envelope> GetViolatedAndEnvelope(const ObservedWorld& ego_only_world,
                                                const AgentPtr& other_agent,
                                                const std::shared_ptr<BaseEvaluator>& evaluator,
                                                const double min_planning_time) {
  #ifdef RSS
  auto rss_evaluator = std::dynamic_pointer_cast<EvaluatorRSS>(evaluator);
  if (rss_evaluator) {
    const auto ego_world_cloned = std::dynamic_pointer_cast<ObservedWorld>(ego_only_world.Clone());
    ego_world_cloned->AddAgent(other_agent);
    auto eval_res = boost::get<std::optional<bool>>(rss_evaluator->Evaluate(*ego_world_cloned));
    const auto& rss_response = rss_evaluator->GetRSSProperResponse();
    const auto& acc_restrictions_rss = rss_response.accelerationRestrictions;
    VLOG(4) << "RSS Response: " << rss_response;
    const auto& acc_restrictions = ConvertRestrictions(min_planning_time, acc_restrictions_rss, *ego_world_cloned,
                      false, Polygon(), 0.1);
    return std::make_pair(*eval_res, acc_restrictions.second);
  }
  #endif
  return std::pair(false, Envelope());
}

EnvelopeProbabilityPair GetEnvelopeProbabilityPair(
  EnvelopeProbabilityList envelope_probability_list,
  const Probability& violation_threshold,
  const std::vector<double> iso_discretizations) {
  auto GetProbabilityRange = [&](std::size_t iso_prob_idx) {
    return iso_prob_idx == 0 ? iso_discretizations.at(0) : iso_discretizations.at(iso_prob_idx) - iso_discretizations.at(iso_prob_idx-1);
  };
  Envelope probabilistic_envelope = envelope_probability_list.at(0).first;
  Probability current_envelope_risk = GetProbabilityRange(envelope_probability_list.at(0).second);
  for(auto env_prob_it = std::next(envelope_probability_list.begin());
    env_prob_it != envelope_probability_list.end(); ++env_prob_it ) {
    const auto iso_prob_idx = env_prob_it->second;
    Probability probability_range = GetProbabilityRange(iso_prob_idx);
    if(current_envelope_risk + probability_range > violation_threshold) {
      break;
    }
    current_envelope_risk = current_envelope_risk + probability_range;
    probabilistic_envelope = env_prob_it->first;
  }
 return EnvelopeProbabilityPair(probabilistic_envelope, current_envelope_risk);
}


EnvelopeProbabilityPair CalculateProbabilisticEnvelope(
  EnvelopeProbabilityList& envelope_probability_list,
  const Probability& violation_threshold,
  const std::vector<double> iso_discretizations) {

  auto mm_env_value = MinMaxEnvelopeValues(envelope_probability_list);

  auto lat_min_pair = GetEnvelopeProbabilityPair(
    mm_env_value[0], violation_threshold, iso_discretizations);
  auto lat_max_pair = GetEnvelopeProbabilityPair(
    mm_env_value[1], violation_threshold, iso_discretizations);
  auto lon_min_pair = GetEnvelopeProbabilityPair(
    mm_env_value[2], violation_threshold, iso_discretizations);
  auto lon_max_pair = GetEnvelopeProbabilityPair(
    mm_env_value[3], violation_threshold, iso_discretizations);

  return EnvelopeProbabilityPair(
    Envelope{
      lat_min_pair.first.lat_acc_min,
      lat_min_pair.first.lat_acc_max,
      lon_min_pair.first.lon_acc_min,
      lon_min_pair.first.lon_acc_max}, 0.);
}

Probability CalculateExpectedViolation(const ViolationProbabilityList& violation_probability_list, const std::vector<double> iso_discretizations) {
  auto GetProbabilityRange = [&](std::size_t iso_prob_idx) {
    return iso_prob_idx == 0 ? iso_discretizations.at(0) : iso_discretizations.at(iso_prob_idx) - iso_discretizations.at(iso_prob_idx-1);
  };
  Probability violation_risk = 0.0;
  for(const auto& violation_probability : violation_probability_list) {
    violation_risk += GetProbabilityRange(violation_probability.second);
  }
  return violation_risk;
}

Trajectory BehaviorSimplexProbabilisticEnvelope::Plan(
    double min_planning_time, const world::ObservedWorld& observed_world) {
  SetBehaviorStatus(BehaviorStatus::VALID);

  // Collect Worst-Case Envelopes over all other agents
  const auto ego_agent = observed_world.GetEgoAgent();
  auto ego_only_world = std::dynamic_pointer_cast<ObservedWorld>(observed_world.Clone());
  ego_only_world->ClearAgents();
  ego_only_world->AddAgent(ego_agent);
  const auto observer_covariance = GetObserverCovariance(observed_world);

  State ego_state = ego_agent->GetCurrentState();
  Point2d ego_position(ego_state(X_POSITION), ego_state(Y_POSITION));
  const Polygon& ego_polygon = ego_agent->GetPolygonFromState(ego_state);
  const unsigned max_nearest_agents = 3; // 2 + 1 ego agent
  const auto nearby_agents =
      observed_world.GetNearestAgents(ego_position, max_nearest_agents);

  EnvelopeProbabilityList envelopes;
  ViolationProbabilityList violations;
  for (const auto& other_agent : nearby_agents) {
    if(other_agent.first == observed_world.GetEgoAgentId()) continue;

    auto envelopes_violations = CalculateAgentsWorstCaseEnvelopes(*ego_only_world, other_agent.second,
                                                     iso_probability_discretizations_, observer_covariance,
                                                      angular_discretization_, rss_evaluator_, min_planning_time);
    auto& agent_envelopes = std::get<0>(envelopes_violations);
    auto& agent_violations = std::get<1>(envelopes_violations);
    MoveAppend(agent_envelopes, envelopes);
    MoveAppend(agent_violations, violations);
  }

  // Calculate probablistic envelope and expected violation
  current_probabilistic_envelope_ =
           CalculateProbabilisticEnvelope(envelopes, violation_threshold_, iso_probability_discretizations_);
  current_expected_safety_violation_ = CalculateExpectedViolation(violations, iso_probability_discretizations_);

  // Handling switching condition and envelope restriction
  Action last_action;
  dynamic::Trajectory last_traj;
  if (current_expected_safety_violation_ > violation_threshold_) {
    behavior_safety_model_->Plan(min_planning_time, observed_world);
    last_action = behavior_safety_model_->GetLastAction();
    last_traj = behavior_safety_model_->GetLastTrajectory();
    behavior_rss_status_ = BehaviorRSSConformantStatus::SAFETY_BEHAVIOR;
  } else {
    #ifdef RSS
    ApplyRestrictionsToModel(current_probabilistic_envelope_.first,
                               nominal_behavior_model_);
    #endif
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
