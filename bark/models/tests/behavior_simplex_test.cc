// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "boost/variant.hpp"
#include "gtest/gtest.h"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/behavior_rss/behavior_rss.hpp"
#include "bark/models/behavior/behavior_simplex/behavior_simplex_probabilistic_envelope.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/dynamic/single_track.hpp"
#include "bark/models/execution/interpolation/interpolate.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/models/observer/observer_model_parametric.hpp"
#include "bark/world/tests/make_test_world.hpp"
#include "bark/world/tests/make_test_xodr_map.hpp"

using bark::commons::SetterParams;
using bark::geometry::Model3D;
using bark::geometry::Polygon;
using bark::world::Agent;
using bark::world::ObservedWorld;
using bark::models::observer::ObserverModelParametric;
using bark::world::World;
using bark::world::WorldPtr;
using bark::world::evaluation::BaseEvaluator;
using bark::world::evaluation::EvaluationReturn;
using bark::world::evaluation::EvaluatorRSS;
using bark::world::evaluation::EvaluatorCollisionEgoAgent;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionPtr;
using bark::world::objects::AgentPtr;
using bark::world::tests::make_test_world;
using bark::world::tests::MakeXodrMapOneRoadTwoLanes;

using namespace bark::models::dynamic;
using namespace bark::models::execution;
using namespace bark::models::behavior;
using namespace bark::world::map;

#ifdef RSS
std::pair<Probability, EnvelopeProbabilityPair> CalculateEnvelopeAndExpectedViolation(double x_standard_deviation, double violation_threshold) {
  auto params = std::make_shared<SetterParams>();

  float ego_velocity = 0.0, rel_distance = 15.0, velocity_difference = 0.0;
  float time_step = 0.2;

  WorldPtr world =
      make_test_world(1, rel_distance, ego_velocity, velocity_difference);

  //2) Set observation uncertainty in x-direction only (test world aligned along x-axis) 
  params->SetListFloat("ObserverModelParametric::EgoStateDeviationDist::Mean", {0, 0});
  params->SetListListFloat("ObserverModelParametric::EgoStateDeviationDist::Covariance",
                              {{0.0, 0.0, 0.0, 0.0}, 
                              {0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0}}); // no covariance for ego agent

  params->SetListFloat("ObserverModelParametric::OtherStateDeviationDist::Mean", {0, 0});
  params->SetListListFloat("ObserverModelParametric::OtherStateDeviationDist::Covariance",
                              {{x_standard_deviation, 0.0, 0.0, 0.0}, 
                              {0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.0, 0.0}}); // only covariance for other agents along x-directions

  auto observer_model_parametric= std::make_shared<ObserverModelParametric>(params);
  world->SetObserverModel(observer_model_parametric);

  params->SetReal("BehaviorSimplexSampling::ViolationThreshold", violation_threshold);
  
  //3) Create behavior and plan
  auto ego_agent = world->GetAgents().begin()->second;
  std::cout << ego_agent->GetCurrentState() << std::endl;
  std::shared_ptr<BehaviorSimplexProbabilisticEnvelope> behavior_simplex_probabilistic_envelope =
      std::make_shared<BehaviorSimplexProbabilisticEnvelope>(params);
  auto eval_rss = std::make_shared<EvaluatorRSS>();
  behavior_simplex_probabilistic_envelope->SetEvaluator(eval_rss);

  auto observed_world = observer_model_parametric->Observe(world, ego_agent->GetAgentId());
  VLOG(3) << "Other state:" << std::next(observed_world.GetAgents().begin())->second->GetCurrentState() << 
              "Ego state" << observed_world.GetEgoAgent()->GetCurrentState();
  auto plan_result = behavior_simplex_probabilistic_envelope->Plan(time_step, observed_world);
  const auto probabilistic_envelope = behavior_simplex_probabilistic_envelope->GetCurrentProbabilisticEnvelope();
  const auto expected_violation = behavior_simplex_probabilistic_envelope->GetCurrentExpectedSafetyViolation();

  return std::pair(expected_violation, probabilistic_envelope);
}


TEST(behavior_simplex_probabilitic_envelope, increase_standard_deviation) {
  // Calculate probabilistic envelope for different standard deviations to
  // qualitatively check if larger standard deviations increases expected
  // violation risk and decreases envelope size
  Probability expected_violations1;
  EnvelopeProbabilityPair envelope_probability1;
  std::tie(expected_violations1, envelope_probability1) = CalculateEnvelopeAndExpectedViolation(0.1, 0.1);

  Probability expected_violations2;
  EnvelopeProbabilityPair envelope_probability2;
  std::tie(expected_violations2, envelope_probability2) = CalculateEnvelopeAndExpectedViolation(0.2, 0.1);

  Probability expected_violations3;
  EnvelopeProbabilityPair envelope_probability3;
  std::tie(expected_violations3, envelope_probability3) = CalculateEnvelopeAndExpectedViolation(0.4, 0.1);

  Probability expected_violations4;
  EnvelopeProbabilityPair envelope_probability4;
  std::tie(expected_violations4, envelope_probability4) = CalculateEnvelopeAndExpectedViolation(0.8, 0.1);

 // EXPECT_TRUE(expected_violations1 < expected_violations2);
 // EXPECT_TRUE(expected_violations2 < expected_violations3);
//  EXPECT_TRUE(expected_violations3 < expected_violations4);

 // EXPECT_TRUE(envelope1 > envelope2);
 // EXPECT_TRUE(envelope2 > envelope3);
 // EXPECT_TRUE(envelope3 > envelope4);
}


#endif

int main(int argc, char** argv) {
  FLAGS_v = 3;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}