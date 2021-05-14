# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


import unittest
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import os
import numpy as np
from bark.runtime.scenario.scenario_generation.deterministic \
  import DeterministicScenarioGeneration
from bark.runtime.scenario.scenario_generation.scenario_generation \
  import ScenarioGeneration

from bark.core.world.goal_definition import GoalDefinition, GoalDefinitionPolygon
from bark.core.geometry import *
from bark.core.world import World, ObservedWorld
from bark.runtime.commons.parameters import ParameterServer
from bark.runtime.runtime import Runtime
from bark.runtime.viewer.matplotlib_viewer import MPViewer
from bark.core.models.behavior import BehaviorModel, BehaviorDynamicModel
from bark.core.models.dynamic import SingleTrackModel
from bark.core.models.observer import *
from bark.core.models.behavior import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.geometry.standard_shapes import *
from bark.core.world.agent import *

# NOTE: this is testing the PyObserverModel wrapping
class PythonObserverModel(ObserverModel):
  def __init__(self,
               params = None):
    ObserverModel.__init__(self, params)
    self._params = params

  def Observe(self, world, agent_id):
    # NOTE: returns a vector as this could return mult. observed worlds
    observed_world = ObservedWorld(world, agent_id)
    return observed_world


def GetParamServerAndWorld():
  param_server = ParameterServer(
    filename=os.path.join(
      os.path.dirname(__file__),
      "../../runtime/tests/data/deterministic_scenario.json"))
      
  mapfile = os.path.join(
    os.path.dirname(__file__),
    "../../runtime/tests/data/city_highway_straight.xodr")
  
  param_server["Scenario"]["Generation"]["DeterministicScenarioGeneration"]["MapFilename"] = mapfile
  scenario_generation = DeterministicScenarioGeneration(num_scenarios=3,
                                                        random_seed=0,
                                                        params=param_server)
  viewer = MPViewer(params=param_server,
                    follow_agent_id=False,
                    use_world_bounds=True)
  scenario, idx = scenario_generation.get_next_scenario()
  world = scenario.GetWorldState()
  return world, param_server


class PyObserverModelTests(unittest.TestCase):
  # def test_observer_model_none(self):
  #   world, param_server = GetParamServerAndWorld()
  #   # NOTE: create and assign ObserverModelNone
  #   observer_model = ObserverModelNone(param_server)
  #   world.observer_model = observer_model
  #   world.Step(0.2)
  #   assert(world.observer_model == observer_model)
    
  # def test_py_observer_model_none(self):
  #   world, param_server = GetParamServerAndWorld()
  #   # NOTE: create and assign PythonObserverModel
  #   observer_model = PythonObserverModel(param_server)
  #   world.observer_model = observer_model
  #   world.Step(0.2)
  #   assert(world.observer_model == observer_model)

  # def test_observer_model_parametric(self):
  #   world, param_server = GetParamServerAndWorld()
  #   # NOTE: create and assign PythonObserverModel
  #   observer_model = ObserverModelParametric(param_server)
  #   world.observer_model = observer_model
  #   world.Step(0.2)
  #   assert(world.observer_model == observer_model)
  
  def test_2d_points_on_sphere(self):
    # https://stats.stackexchange.com/questions/361017/proper-way-of-estimating-the-covariance-error-ellipse-in-2d
    # https://stackoverflow.com/questions/12301071/multidimensional-confidence-intervals/39749274#39749274

    # NOTE: to "verify" the confidence ellipses the results of https://stackoverflow.com/questions/25718363/how-to-plot-bivariate-normal-distribution-with-expanding-ellipses
    #       are "reproduced"
    fig = plt.figure()
    for p in [.05, .25, .50, .75, .95]:
      # p = 0.95
      cov = np.array([
        [9., 3.],
        [3., 4.]])
      perm_angles = GetAllPermutatedAngles([.3])
      pts = GetPointsOnSphere(cov, perm_angles, p)
      pts = np.array(pts)
      
      # C = 5.99146
      # ellipsis_eq = lambda x,y : x**2/(C*2) + y**2/(C*4)
      # for pt in pts:
      #   print("Value", ellipsis_eq(pt[0], pt[1]))
      
      pts = np.vstack((pts, pts[0,:]))
      plt.plot(pts[:, 0] + 1, pts[:, 1] + 2)
      # plt.axis("equal")
    plt.gca().set_xlim([])
    plt.gca().set_ylim([])
    plt.show()

    
  # def test_3d_points_on_sphere(self):
  #   cov = np.array([
  #     [3., 0., 0.],
  #     [0., 1., 0.],
  #     [0., 0., 2.]])
  #   perm_angles = GetAllPermutatedAngles([.3, .3])
  #   pts = GetPointsOnSphere(cov, perm_angles, .98)
  #   pts = np.array(pts)
    
  #   print(pts.shape)    
  #   fig = plt.figure()
  #   ax = Axes3D(fig)
  #   ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "r*")


  #   pts = GetPointsOnSphere(0.5*cov, perm_angles, .98)
  #   pts = np.array(pts)
  #   ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "b*")
  #   # plt.axis("equal")
  #   # plt.show()
    
  # def test_4d_points_on_sphere(self):
  #   cov = np.array([
  #     [3., 0., 0., 0.],
  #     [0., 1., 0., 0.],
  #     [0., 0., 2., 0.],
  #     [0., 0., 0., 3.]])
  #   perm_angles = GetAllPermutatedAngles([.5, .5, .5])
  #   pts = GetPointsOnSphere(cov, perm_angles, 0.98)
  #   pts = np.array(pts)
    
  #   print(pts.shape)    
  #   fig = plt.figure()
  #   ax = Axes3D(fig)
  #   ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "r*")
  #   ax.plot(pts[:, 1], pts[:, 2], pts[:, 3], "b*")
  #   # ax.set_xlim([-1, 1])
  #   # ax.set_ylim([-1, 1])
  #   # ax.set_zlim([-1, 1])
  #   # plt.show()
    
  # def test_agent_state_isoline(self):
  #   params = ParameterServer()
  #   behavior = BehaviorIDMClassic(params)
  #   execution = ExecutionModelInterpolate(params)
  #   dynamic = SingleTrackModel(params)
  #   shape = CarLimousine()
  #   init_state = np.array([0, 0, 0, 0, 5])
  #   goal_polygon = Polygon2d([0, 0, 0],[Point2d(-1,-1),Point2d(-1,1),Point2d(1,1), Point2d(1,-1)])
  #   goal_definition = GoalDefinitionPolygon(goal_polygon)
  #   agent = Agent(init_state, behavior, dynamic, execution, shape, params.AddChild("agent"), goal_definition)
    
  #   cov = np.array([
  #     [3., 0., 0.],
  #     [0., 1., 0.],
  #     [0., 0., 5.]])
  #   delta_theta = [0.5, 0.5]
  #   agent_list = ObserveAtIsoLine(agent, delta_theta, cov, 0.98)
    
    
  #   fig = plt.figure()
  #   ax = Axes3D(fig)
  #   ax.set_title("Agent states")
  #   for agent in agent_list:
  #     state = agent.state
  #     plt.plot([state[1]], [state[2]], state[3], 'r*')
  #   plt.show()
    


if __name__ == '__main__':
  unittest.main()