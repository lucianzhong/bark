# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import unittest
import os
import numpy as np
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

# parameters
param_server = ParameterServer(filename=Data.params_data("highway_merge_configurable"))
param_server["BehaviorSimplexProbabilisticEnvelope"]["MinVehicleRearDistance"] = 4.
param_server["BehaviorSimplexProbabilisticEnvelope"]["MinVehicleFrontDistance"] = 2.
param_server["BehaviorSimplexProbabilisticEnvelope"]["TimeKeepingGap"] = 0.
param_server["World"]["LateralDifferenceThreshold"] = 2.0

# param_server["Visualization"]["Evaluation"]["DrawRssDebugInfo"] = True
# param_server["Visualization"]["Evaluation"]["DrawRssSafetyResponses"] = True
param_server["Visualization"]["Evaluation"]["DrawEgoRSSSafetyResponses"] = True


# custom lane configuration that sets a different behavior model
# and sets the desired speed for the behavior
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
        behavior_model = BehaviorSimplexProbabilisticEnvelope(params)
        self._count += 1
        return behavior_model

def CalculateEnvelopeAndExpectedViolation(x_standard_deviation = 0.2, violation_threshold = 0.1):  
  param_server["ObserverModelParametric"]["EgoStateDeviationDist"]["Mean"] = np.array([0., 0.], dtype=np.float32)
  param_server["ObserverModelParametric"]["EgoStateDeviationDist"]["Covariance"] = \
                              np.array([[0.0, 0.0, 0.0, 0.0], 
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
  
  param_server["ObserverModelParametric"]["OtherStateDeviationDist"]["Mean"] = {0,0}
  param_server["ObserverModelParametric"]["OtherStateDeviationDist"]["Covariance"] = \
                              np.array([[x_standard_deviation, 0.0, 0.0, 0.0], 
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0]], dtype=np.float32)

  param_server["BehaviorSimplexProbabilisticEnvelope"]["ViolationThreshold"] = violation_threshold
  observer_model_parametric = ObserverModel(param_server)

  # TODO
  # world->SetObserverModel(observer_model_parametric);

  # configure both lanes of the highway. the right lane has one controlled agent
  left_lane = HighwayLaneCorridorConfig(params=param_server,
                                        road_ids=[16],
                                        lane_corridor_id=0)
  right_lane = HighwayLaneCorridorConfig(params=param_server,
                                        road_ids=[16],
                                        lane_corridor_id=1,
                                        controlled_ids=True)


  # create 1 scenarios
  scenarios = \
      ConfigWithEase(
          num_scenarios=1,
          map_file_name=Data.xodr_data("city_highway_straight"),
          random_seed=0,
          params=param_server,
          lane_corridor_configs=[left_lane, right_lane])

  # viewer
  # viewer = MPViewer(params=param_server,
  #                   x_range=[-75, 75],
  #                   y_range=[-75, 75],
  #                   follow_agent_id=True)

  # sim_step_time = param_server["simulation"]["step_time",
  #                                           "Step-time used in simulation",
  #                                           0.05]
  # sim_real_time_factor = param_server["simulation"][
  #     "real_time_factor",
  #     "execution in real-time or faster",
  #     0.5]

  viewer = VideoRenderer(renderer=viewer,
                        world_step_time=sim_step_time,
                        fig_path="/tmp/video")

  # gym like interface
  env = Runtime(step_time=0.2,
                viewer=viewer,
                scenario_generator=scenarios,
                render=False)


  # Defining vehicles dynamics for RSS

  # Input format:
  # [longitudinal max acceleration, longitudinal max braking, longitudinal min acceleration,
  # longitudinal min brake correct, lateral max acceleration, lateral min braking,
  # lateral flucatuation_margin, agent response time]
  #
  # Detailed explanation please see:
  # https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion


  param_server["EvaluatorRss"]["MapFilename"] = Data.xodr_data("city_highway_straight")

  # run 1 scenario
  for episode in range(0, 1):
      env.reset()
      current_world = env._world
      current_world.SetObserverModel(observer_model_parametric)
      eval_agent_id = env._scenario._eval_agent_ids[0]
      observed_world = observer_model_parametric.Observe(current_world, eval_agent_id)
      ego_agent = current_world.agents[eval_agent_id] 
      ego_agent.behavior_model = \
        BehaviorSimplexProbabilisticEnvelope(param_server)
      evaluator_rss = EvaluatorRSS(eval_agent_id, param_server)

      current_world.AddEvaluator("rss", evaluator_rss)

      # step each scenario 40 times
      for step in range(0, 1):
          env.step()
          print_rss_safety_response(evaluator_rss, current_world)
          time.sleep(sim_step_time / sim_real_time_factor)

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

class PyBehaviorModelTests(unittest.TestCase):
  def test_python_model(self):

    x_standard_deviation = 0.2
    violation_threshold = 0.1
    param_server["ObserverModelParametric"]["EgoStateDeviationDist"]["Mean"] = np.array([0., 0.], dtype=np.float32)
    param_server["ObserverModelParametric"]["EgoStateDeviationDist"]["Covariance"] = \
                                np.array([[0.0, 0.0, 0.0, 0.0], 
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0]], dtype=np.float32)
    
    param_server["ObserverModelParametric"]["OtherStateDeviationDist"]["Mean"] = {0,0}
    param_server["ObserverModelParametric"]["OtherStateDeviationDist"]["Covariance"] = \
                                np.array([[x_standard_deviation, 0.0, 0.0, 0.0], 
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0]], dtype=np.float32)

    param_server["BehaviorSimplexProbabilisticEnvelope"]["ViolationThreshold"] = violation_threshold
    observer_model_parametric = ObserverModel(param_server)

    # TODO
    # world->SetObserverModel(observer_model_parametric);

    # configure both lanes of the highway. the right lane has one controlled agent
    left_lane = HighwayLaneCorridorConfig(params=param_server,
                                          road_ids=[16],
                                          lane_corridor_id=0)
    right_lane = HighwayLaneCorridorConfig(params=param_server,
                                          road_ids=[16],
                                          lane_corridor_id=1,
                                          controlled_ids=True)


    # create 1 scenarios
    scenarios = \
        ConfigWithEase(
            num_scenarios=1,
            map_file_name=Data.xodr_data("city_highway_straight"),
            random_seed=0,
            params=param_server,
            lane_corridor_configs=[left_lane, right_lane])

    # viewer
    viewer = MPViewer(params=param_server,
                      x_range=[-75, 75],
                      y_range=[-75, 75],
                      follow_agent_id=True)

    sim_step_time = param_server["simulation"]["step_time",
                                              "Step-time used in simulation",
                                              0.05]
    sim_real_time_factor = param_server["simulation"][
        "real_time_factor",
        "execution in real-time or faster",
        0.5]

    viewer = VideoRenderer(renderer=viewer,
                          world_step_time=sim_step_time,
                          fig_path="/tmp/video")

    # gym like interface
    env = Runtime(step_time=0.2,
                  viewer=viewer,
                  scenario_generator=scenarios,
                  render=True)


    # Defining vehicles dynamics for RSS

    # Input format:
    # [longitudinal max acceleration, longitudinal max braking, longitudinal min acceleration,
    # longitudinal min brake correct, lateral max acceleration, lateral min braking,
    # lateral flucatuation_margin, agent response time]
    #
    # Detailed explanation please see:
    # https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion


    param_server["EvaluatorRss"]["MapFilename"] = Data.xodr_data("city_highway_straight")

    # change amit
    # run 3 scenarios
    for episode in range(0, 1):
        env.reset()
        current_world = env._world
        current_world.SetObserverModel(observer_model_parametric)
        eval_agent_id = env._scenario._eval_agent_ids[0]
        observed_world = observer_model_parametric.Observe(current_world, eval_agent_id)
        ego_agent = current_world.agents[eval_agent_id] 
        ego_agent.behavior_model = \
          BehaviorSimplexProbabilisticEnvelope(param_server)
        evaluator_rss = EvaluatorRSS(eval_agent_id, param_server)

        current_world.AddEvaluator("rss", evaluator_rss)

        # step each scenario 40 times
        for step in range(0, 4):
            env.step()
            print_rss_safety_response(evaluator_rss, current_world)
            time.sleep(sim_step_time / sim_real_time_factor)

            envelope = ego_agent.behavior_model.GetCurrentProbabilisticEnvelope()
            expected_violation = ego_agent.behavior_model.GetCurrentExpectedSafetyViolation()

            print("envelope: ", envelope)
            print("\n expected_violation: ", expected_violation)

    # viewer.export_video(filename="/tmp/highway_rss", remove_image_dir=False)


# class PyBehaviorModelTests(unittest.TestCase):
#   def test_python_model(self):
#     param_server = ParameterServer(
#       filename= os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/deterministic_scenario.json"))
#     param_server
    
#     mapfile = os.path.join(os.path.dirname(__file__),"../../runtime/tests/data/city_highway_straight.xodr")
#     param_server["Scenario"]["Generation"]["DeterministicScenarioGeneration"]["MapFilename"] = mapfile
#     scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
#                                                           random_seed=0,
#                                                           params=param_server)
#     viewer = MPViewer(params=param_server,
#                       follow_agent_id=False,
#                       use_world_bounds=True)
#     scenario, idx = scenario_generation.get_next_scenario()
#     world = scenario.GetWorldState()
#     single_track_model = SingleTrackModel(param_server)
#     behavior_model = PythonBehaviorModelWrapper(
#       single_track_model, param_server)
#     world.GetAgent(0).behavior_model = behavior_model
#     world.GetAgent(0).behavior_model.SetLastAction(
#       np.array([1., 1.], dtype=np.float32))
#     world.Step(0.2)




if __name__ == '__main__':
  unittest.main()