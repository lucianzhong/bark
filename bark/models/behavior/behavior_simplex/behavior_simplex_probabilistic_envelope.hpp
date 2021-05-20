// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_PROBABILISTIC_ENVELOPE_HPP_
#define BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_PROBABILISTIC_ENVELOPE_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/world/world.hpp"


#include "bark/world/evaluation/rss/safety_polygon.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::BehaviorSafety;
using bark::models::dynamic::AccelerationLimits;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::World;
using world::evaluation::BaseEvaluator;
using world::evaluation::ComputeSafetyPolygon;
using world::evaluation::SafetyPolygon;
using world::objects::AgentId;
using world::objects::AgentPtr;
using bark::commons::Probability;

typedef AccelerationLimits Envelope;
typedef std::pair<Envelope, Probability> EnvelopeProbabilityPair;
typedef std::vector<EnvelopeProbabilityPair> EnvelopeProbabilityList;
typedef std::pair<bool, Probability> ViolationProbabilityPair;
typedef std::vector<ViolationProbabilityPair> ViolationProbabilityList;

Eigen::MatrixXd GetObserverCovariance(const ObservedWorld& observed_world);

std::pair<EnvelopeProbabilityList, ViolationProbabilityList>
                                          CalculateAgentsWorstCaseEnvelopes(const ObservedWorld& ego_only_world,
                                                                            const AgentPtr& other_agent,
                                                                            const std::vector<double> iso_discretizations,
                                                                            const Eigen::MatrixXd& observer_covariance,
                                                                            const std::vector<double> angular_discretization,
                                                                            std::shared_ptr<BaseEvaluator>& evaluator,
                                                                            const double min_planning_time);

std::pair<bool, Envelope> GetViolatedAndEnvelope(const ObservedWorld& ego_only_world,
                                                const AgentPtr& other_agent,
                                                const std::shared_ptr<BaseEvaluator>& evaluator,
                                                const double min_planning_time);

void SortEnvelopes(EnvelopeProbabilityList& envelope_probability_list);

EnvelopeProbabilityPair CalculateProbabilisticEnvelope(EnvelopeProbabilityList& envelope_probability_list, const Probability& violation_threshold,
                                                      const std::vector<double> iso_discretizations);

Probability CalculateExpectedViolation(const ViolationProbabilityList& violation_probability_list, const std::vector<double> iso_discretizations);


class BehaviorSimplexProbabilisticEnvelope : public BehaviorRSSConformant {
 public:
  explicit BehaviorSimplexProbabilisticEnvelope(const commons::ParamsPtr& params)
      : BehaviorRSSConformant(params),
        iso_probability_discretizations_(params->GetListFloat("BehaviorSimplexProbabilisticEnvelope::IsoProbalityDiscretizations",
                                                         "List with iso probaiblities", {0.1, 0.2, 0.4, 0.8, 1.0})),
        angular_discretization_(params->GetListFloat("BehaviorSimplexProbabilisticEnvelope::AngularDiscretization",
                                            "List with delta angles for sampling iso lines, one less than covariance dimension", {0.1, 0.1, 0.1})),
        current_probabilistic_envelope_(),
        violation_threshold_(params->GetReal("BehaviorSimplexProbabilisticEnvelope::ViolationThreshold", "Maximum allowed probability"
        " of violating RSS in current or next state", 0.01)) {}

  virtual ~BehaviorSimplexProbabilisticEnvelope() {}

  Trajectory Plan(double min_planning_time,
                  const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

    //double current_expected_safety_violation_;
  double GetCurrentExpectedSafetyViolation() const { return current_expected_safety_violation_;}

  EnvelopeProbabilityPair GetCurrentProbabilisticEnvelope() const { return current_probabilistic_envelope_; }

 private:
  std::vector<double>  iso_probability_discretizations_;
  std::vector<double>  angular_discretization_;
  double current_expected_safety_violation_;
  EnvelopeProbabilityPair current_probabilistic_envelope_;

  double violation_threshold_;
};

inline std::shared_ptr<BehaviorModel> BehaviorSimplexProbabilisticEnvelope::Clone() const {
  std::shared_ptr<BehaviorSimplexProbabilisticEnvelope> model_ptr =
      std::make_shared<BehaviorSimplexProbabilisticEnvelope>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_PROBABILISTIC_ENVELOPE_HPP_
