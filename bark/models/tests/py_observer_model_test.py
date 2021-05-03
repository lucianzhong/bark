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
    
  def test_points_on_sphere(self):
    cov = np.array([
      [3., 0., 0., 0.],
      [0., 1., 0., 0.],
      [0., 0., 2., 0.],
      [0., 0., 0., 3.]])
    perm_angles = GetAllPermutatedAngles([.5, .5, .5])
    pts = GetPointsOnSphere(cov, perm_angles, 1.)
    pts = np.array(pts)
    
    print(pts.shape)    
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "r*")
    ax.plot(pts[:, 1], pts[:, 2], pts[:, 3], "b*")
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    plt.show()
    
    


if __name__ == '__main__':
  unittest.main()