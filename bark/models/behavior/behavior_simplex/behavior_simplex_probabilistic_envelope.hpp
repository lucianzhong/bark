// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_SAMPLING_HPP_
#define BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_SAMPLING_HPP_     

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
using world::evaluation::BaseEvaluator;
using world::evaluation::ComputeSafetyPolygon;
using world::evaluation::SafetyPolygon;
using world::objects::AgentId;
using bark::commons::Probability;
#ifdef RSS
using bark::world::evaluation::EvaluatorRSS;
#endif

typedef AccelerationLimits Envelope;
typedef std::pair<Envelope, Probability> EnvelopeProbabilityPair;
typedef std::vector<EnvelopeProbabilityPair> EnvelopeProbabilityList;
typedef std::pair<bool, Probability> ViolationProbabilityPair;
typedef std::vector<ViolationProbabilityPair> ViolationProbabilityList;

Eigen::MatrixXd GetObserverCovariance(const ObservedWord& observed_world);

std::pair<EnvelopeProbabilityList, ViolationProbabilityList> 
                                          CalculateAgentsWorstCaseEnvelopes(const AgentPtr& ego_agent,
                                                                            const AgentPtr& other_agent, 
                                                                            const std::vector<double> iso_discretizations,
                                                                            const Eigen::MatrixXd& observer_covariance);

EnvelopeProbabilityPair CalculateProbabilisticEnvelope(const EnvelopeProbabilityList& envelope_probability_list, const Probability& violation_threshold); 

Probability CalculateExpectedViolation(const ViolationProbabilityList& violation_probability_list);


class BehaviorSimplexProbabilisticEnvelope : public BehaviorRSSConformant {
 public:
  explicit BehaviorSimplexProbabilisticEnvelope(const commons::ParamsPtr& params)
      : BehaviorRSSConformant(params),
        iso_probability_discretizations_(params->GetListFloat("BehaviorSimplexProbabilisticEnvelope::IsoProbalityDiscretizations", "List with iso probaiblities", {0.1, 0.2, 0.4, 0.8})),
        violation_threshold_(params->GetReal("BehaviorSimplexProbabilisticEnvelope::ViolationThreshold", "Maximum allowed probability"
        " of violating RSS in current or next state", 0.01)) {}

  virtual ~BehaviorSimplexSampling() {}

  Trajectory Plan(double min_planning_time,
                  const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

    //double current_expected_safety_violation_;
  double GetCurrentExpectedSafetyViolation() const { return current_expected_safety_violation_;}

 private:
  std::vector<double>  iso_probability_discretizations_;
  double current_expected_safety_violation_;
  double violation_threshold_;
};

inline std::shared_ptr<BehaviorModel> BehaviorSimplexSampling::Clone() const {
  std::shared_ptr<BehaviorSimplexSampling> model_ptr =
      std::make_shared<BehaviorSimplexSampling>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_BEHAVIOR_SIMPLEX_BEHAVIOR_SIMPLEX_SAMPLING_HPP_
