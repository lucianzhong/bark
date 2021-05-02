// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_OBSERVER_UTILS_ELLIPSIS_HPP_
#define BARK_MODELS_OBSERVER_UTILS_ELLIPSIS_HPP_

#include <memory>
#include <Eigen/Eigenvalues> 
#include "bark/commons/base_type.hpp"

namespace bark {

// forward declarations
namespace world {
namespace objects {
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
class World;
typedef std::shared_ptr<World> WorldPtr;
}  // namespace world

namespace models {
namespace observer {
namespace utils {


void expand(
  const std::map<int, std::vector<double>>& dim_angles,
  std::vector<int> curr_ids,
  std::vector<std::vector<double>>& permutated_angles,
  int exp_axis = 0,
  bool add=true) {
  
  // generate angle based on curr_ids
  std::vector<double> pangle;
  for (int i = 0; i < curr_ids.size(); i++) {
    double angle = dim_angles.at(i)[curr_ids[i]];
    pangle.push_back(angle);
  }
  permutated_angles.push_back(pangle);


  if (curr_ids[exp_axis] < dim_angles.at(exp_axis).size() - 1) {
    // cols
    if (exp_axis < curr_ids.size() - 1) {
      expand(dim_angles, curr_ids, permutated_angles, exp_axis + 1);
    }
    curr_ids[exp_axis] += 1;
    // rows
    expand(dim_angles, curr_ids, permutated_angles, exp_axis);

    // last cols
    if (curr_ids[exp_axis] == dim_angles.at(exp_axis).size() - 1 &&
        exp_axis == 0) {
      expand(dim_angles, curr_ids, permutated_angles, exp_axis+1);
    }
  }

  return; 
}

std::vector<std::vector<double>> GetAllPermutatedAngles(
  const std::vector<double>& delta_theta) {
  // initialize angles
  std::map<int, std::vector<double>> dim_angles;
  for (int i = 0; i < delta_theta.size(); i++) {
    std::vector<double> angles;
    for (double t = 0; t <= 2*M_PI; t+=delta_theta[i]) {
      angles.push_back(t);
    }
    dim_angles[i] = angles;
  }

  // get permutated angles
  std::vector<int> curr_ids(delta_theta.size(), 0);
  std::vector<std::vector<double>> permutated_angles;
  expand(dim_angles, curr_ids, permutated_angles);

  // return all combination of angles
  return permutated_angles;
}

using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
Matrix ComputeEVs(const Matrix& cov) {
  // EigenSolver<Matrix> es(cov);
  // Matrix evs = es.eigenvectors();
  // return evs;
}


// ObserveAtIsoLine(AgentPtr (o_t), angular_delta (vector with dimensions of multivariate distribution), P_ISO)  -> std::vector (2pi/angular_delta_dim12pi/angular_delta_dim22pi/angular_delta_dim3*2pi/angular_delta_dim3) (Patrick)
// std::vector<AgentPtr> ObserveAtIsoLine(
//   const AgentPtr& agent, std::vector<double> delta_theta, double p_iso) {

// }

}
}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_UTILS_ELLIPSIS_HPP_
