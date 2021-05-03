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
  int exp_axis = 0) {
  
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
        exp_axis == 0 && exp_axis+1 < curr_ids.size()) {
      expand(dim_angles, curr_ids, permutated_angles, exp_axis+1);
    }
  }
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

using MatrixD = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixC = Eigen::Matrix<
  std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>;
std::pair<MatrixC, MatrixC> ComputeEVs(const MatrixD& cov) {
  Eigen::EigenSolver<MatrixD> es(cov);
  MatrixC e_vec = es.eigenvectors();
  MatrixC e_values = es.eigenvalues();
  return std::make_pair(e_vec, e_values);
}

std::vector<std::vector<double>> GetPointsOnSphere(
  const MatrixD& cov,
  const std::vector<std::vector<double>>& permutated_angles,
  double p_iso = 0.1) {
 
  auto e_vec_val = ComputeEVs(cov);
  auto evec_mat = std::get<0>(e_vec_val).real();
  auto eval_mat = std::get<1>(e_vec_val).real();
  // https://www.michaelchughes.com/blog/2013/01/why-contours-for-multivariate-gaussian-are-elliptical/
  double a_sq = p_iso/evec_mat(0, 0)/eval_mat(0);
  double b_sq = p_iso/evec_mat(1, 1)/eval_mat(1);
  double c_sq = p_iso/evec_mat(2, 2)/eval_mat(2);

  // std::cout << "a_sq: " << a_sq << ", b_sq: " << b_sq  << ", c_sq: " << c_sq << std::endl;
  std::vector<std::vector<double>> points_on_sphere;
  for (auto d : permutated_angles) {
    // spherical coordinates (R=1 plots unitsphere)
    // https://stackoverflow.com/questions/36816537/spherical-coordinates-plot-in-matplotlib
    double x = a_sq*sin(d[0])*cos(d[1]); 
    double y = b_sq*sin(d[0])*sin(d[1]); 
    double z = c_sq*cos(d[0]); 
    points_on_sphere.push_back({x,y,z});
  }
  return points_on_sphere;
}

// ObserveAtIsoLine(AgentPtr (o_t), angular_delta (vector with dimensions of multivariate distribution), P_ISO)  -> std::vector (2pi/angular_delta_dim12pi/angular_delta_dim22pi/angular_delta_dim3*2pi/angular_delta_dim3) (Patrick)
std::vector<AgentPtr> ObserveAtIsoLine(
  const AgentPtr& agent, std::vector<double> delta_theta, const MatrixD& cov, double p_iso) {

  std::vector<std::vector<double>> permutated_angles =
    GetAllPermutatedAngles(delta_theta);

  std::vector<std::vector<double>> pts_on_sphere = 
    GetPointsOnSphere(cov, permutated_angles, p_iso);

  for (const auto& pt: pts_on_sphere) {
    // auto agent = agent.Clone();
    // modify state here

  }

}

}
}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_UTILS_ELLIPSIS_HPP_
