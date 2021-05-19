# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

try:
    import debug_settings
except:
    pass
import unittest
import os
import numpy as np
import logging
import matplotlib.pyplot as plt
logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

logging.info("Running on process with ID: {}".format(os.getpid()))

from bark.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from bark.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration

from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.core.geometry import *
from bark.core.world import World
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.runtime import Runtime
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.core.models.behavior import BehaviorModel, BehaviorDynamicModel
from bark.core.models.dynamic import SingleTrackModel

import time
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.runtime.scenario.scenario_generation.config_with_ease import \
    LaneCorridorConfig, ConfigWithEase
from bark.runtime.runtime import Runtime
from bark.examples.paths import Data
from bark.core.models.behavior import *
from bark.core.models.observer import *

try:
    from bark.core.world.evaluation import EvaluatorRSS
except:
    raise ImportError(
        "This example requires building RSS, please run with \"bazel run //examples:highway_rss --define rss=true\"")

class HighwayLaneCorridorConfig(LaneCorridorConfig):
    def __init__(self, params=None, **kwargs):
        super().__init__(params=params, **kwargs)
        self._count = 0

    def velocity(self):
        return np.random.uniform(2., 10.)

    def behavior_model(self, world):
        # agent_params = ParameterServer()
        params = self._params.AddChild(
            "BehaviorSimplexProbabilisticEnvelope"+str(self._count))
        params["BehaviorIDMClassic"]["DesiredVelocity"] = \
            np.random.uniform(5., 10.)
        behavior_model = BehaviorLaneChangeRuleBased(params)
        self._count += 1
        return behavior_model

def CalculateEnvelopeAndExpectedViolation(x_standard_deviation = 0.2, violation_threshold = 0.1):  
  params_parametric = ParameterServer()
  params_parametric["ObserverModelParametric"] \
        ["EgoStateDeviationDist"]["Covariance"] = [[0.0, 0.0, 0.0, 0.0],  #  4 x 4 elements
                                                  [0.0, 0.0, 0.0, 0.0],
                                                  [0.0, 0.0, 0.0, 0.0],
                                                  [0.0, 0.0, 0.0, 0.0]]
  params_parametric["ObserverModelParametric"] \
        ["EgoStateDeviationDist"]["Mean"] =       [0.0, 0.0, 0.0, 0.0] # 4 elements
  params_parametric["ObserverModelParametric"] \
        ["OtherStateDeviationDist"]["Covariance"] = [[x_standard_deviation, 0.0, 0.0, 0.0],  #  4 x 4 elements
                                                  [0.0, 0.01, 0.0, 0.0],
                                                  [0.0, 0.00, 0.001, 0.0],
                                                  [0.0, 0.00, 0.0, 0.05]]
  params_parametric["ObserverModelParametric"] \
      ["OtherStateDeviationDist"]["Mean"] =       [0.0, 0.0, 0.0, 0.0] # 4 elements
  parametric_observer = ObserverModelParametric(params_parametric)

  params_scenario = ParameterServer()
  left_lane = HighwayLaneCorridorConfig(params=params_scenario,
                                        road_ids=[16],
                                        lane_corridor_id=0)
  right_lane = HighwayLaneCorridorConfig(params=params_scenario,
                                        road_ids=[16],
                                        lane_corridor_id=1,
                                        controlled_ids=True)
        
  # create 1 scenarios
  scenarios = \
      ConfigWithEase(
          num_scenarios=1,
          map_file_name=Data.xodr_data("city_highway_straight"),
          random_seed=0,
          params=params_scenario,
          lane_corridor_configs=[left_lane, right_lane])

  params_behavior = ParameterServer()
  params_behavior["EvaluatorRss"]["MapFilename"] = Data.xodr_data("city_highway_straight")
  params_behavior["BehaviorSimplexProbabilisticEnvelope"]["IsoProbalityDiscretizations"] = [0.1, 0.2, 0.4, 0.8]
  params_behavior["BehaviorSimplexProbabilisticEnvelope"]["AngularDiscretization"] = [3.14/2.0, 3.14/2.0, 3.14/2.0]
  params_behavior["BehaviorSimplexProbabilisticEnvelope"]["ViolationThreshold"] = violation_threshold

  # Step scenario once, to invoke planning of probabilistic envelope behavior
  scenario = scenarios.get_scenario(0)
  world = scenario.GetWorldState()
  world.observer_model = parametric_observer
  ego_agent = world.agents[scenario.eval_agent_ids[0]] 
  ego_agent.behavior_model = \
    BehaviorSimplexProbabilisticEnvelope(params_behavior)
  world.Step(0.2)
  envelope = ego_agent.behavior_model.GetCurrentProbabilisticEnvelope()
  expected_violation = ego_agent.behavior_model.GetCurrentExpectedSafetyViolation()

  # print("envelope: ", envelope)
  # print("\n expected_violation: ", expected_violation)
  return (envelope, expected_violation)

def print_rss_safety_response(evaluator_rss, world):
        # Example of using RSS to evaluate the safety situation of the evaluating agent.
        # The evaluating agent is defined with agent_id when initializing EvaluatorRSS.
        # Evaluating with RSS is quite computionally expensive
        print("Overall safety response: ", evaluator_rss.Evaluate(world))
        # print("Pairwise safety response: ",
        #       evaluator_rss.PairwiseEvaluate(world))
        # print("Pairwise directional safety response: ",
        #       evaluator_rss.PairwiseDirectionalEvaluate(world))

class PyProbabilisticEnvelopeBehaviorTests(unittest.TestCase):
  def test_increase_standard_deviation(self):
    envelope, expected_violation = CalculateEnvelopeAndExpectedViolation()
    




if __name__ == '__main__':
  unittest.main()