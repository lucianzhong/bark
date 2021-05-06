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
#include <boost/math/distributions/chi_squared.hpp>
#include "bark/commons/base_type.hpp"

namespace bark {

// forward declarations
namespace world {
namespace objects {
typedef unsigned int AgentId;
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
}  // namespace objects
class ObservedWorld;
class World;
typedef std::shared_ptr<World> WorldPtr;
}  // namespace world

namespace models {
namespace observer {
namespace utils {

using bark::world::objects::Agent;
using bark::world::objects::AgentPtr;
using models::dynamic::State;

/**
 * @brief  Function that recursively computes all angle combinations
 */
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

/**
 * @brief  Returns all permutated angles
 * @note   
 * @param  delta_theta: e.g., [0.25, 0.5]
 * @retval 
 */
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

/**
 * @brief  Computes Eigenvectors and values
 */
using MatrixD = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MatrixC = Eigen::Matrix<
  std::complex<double>, Eigen::Dynamic, Eigen::Dynamic>;
std::pair<MatrixC, MatrixC> ComputeEVs(const MatrixD& cov) {
  Eigen::EigenSolver<MatrixD> es(cov);
  MatrixC e_vec = es.eigenvectors();
  MatrixC e_values = es.eigenvalues();
  return std::make_pair(e_vec, e_values);
}

/**
 * @brief  Returns all points on an n-dimensional sphere that lie
 *         on the isoline
 * @note   
 * @param  cov: covariance matrix
 * @param  permutated_angles: all angle combinations for n-1 dimensions
 * @param  p_iso: e.g., 0.98 quantile
 * @retval 
 */
std::vector<std::vector<double>> GetPointsOnSphere(
  const MatrixD& cov,
  const std::vector<std::vector<double>>& permutated_angles,
  double p_iso = 0.98) {
 
  auto e_vec_val = ComputeEVs(cov);
  auto evec_mat = std::get<0>(e_vec_val).real();
  auto eval_mat = std::get<1>(e_vec_val).real();
  // std::cout << evec_mat << std::endl;
  // std::cout << eval_mat << std::endl;

  // NOTE: for the isolines
  // this gives us (x/a^2) + ... = C
  // https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
  // https://people.richland.edu/james/lecture/m170/tbl-chi.html
  // degree of freedom is size - 1
  boost::math::chi_squared mydist(eval_mat.size()-1);
  auto C = quantile(mydist, p_iso);
  std::cout << "C = " << C << std::endl;

  std::vector<double> coeffs;
  for (int i = 0; i < eval_mat.size(); i++) {
    // scaled EVs
    // see: https://www.michaelchughes.com/blog/2013/01/why-contours-for-multivariate-gaussian-are-elliptical/
    double coeff = sqrt(C/eval_mat(i));
    coeffs.push_back(coeff);
  }

  std::vector<std::vector<double>> points_on_sphere;
  for (auto d : permutated_angles) {
    // n-spherical coordinates
    // https://math.stackexchange.com/questions/50953/volume-of-region-in-5d-space/51406#51406
    // https://en.wikipedia.org/wiki/N-sphere#Spherical_coordinates
    double sin_chain = 1;
    std::vector<double> hyper_sphere_coord;
    for (int n = 0; n < d.size(); n++){
      double coeff = sin_chain*cos(d[n]);
      hyper_sphere_coord.push_back(coeff);
      sin_chain *= sin(d[n]);
    }
    hyper_sphere_coord.push_back(sin_chain);

    // computed coefficients x spherical coordinates
    std::vector<double> pts;
    for (int i = 0; i < coeffs.size(); i++){
      pts.push_back(coeffs[i]*hyper_sphere_coord[i]);
    }

    // rotate spherical coordinates
    MatrixD eigen_pts(pts.size(), 1);
    for (int i = 0; i < pts.size(); i++) {
      eigen_pts(i, 0) = pts[i];
    }
    auto res = evec_mat.transpose()*eigen_pts;
    std::vector<double> rot_pts;
    for (int i = 0; i < pts.size(); i++) {
      rot_pts.push_back(res(i, 0));
    }

    points_on_sphere.push_back(rot_pts);
  }

  return points_on_sphere;
}

// ObserveAtIsoLine(AgentPtr (o_t), angular_delta (vector with dimensions of multivariate distribution), P_ISO)  -> std::vector (2pi/angular_delta_dim12pi/angular_delta_dim22pi/angular_delta_dim3*2pi/angular_delta_dim3) (Patrick)
std::vector<AgentPtr> ObserveAtIsoLine(
  const AgentPtr& agent, std::vector<double> delta_theta,
  const MatrixD& cov, double p_iso) {

  std::vector<std::vector<double>> permutated_angles =
    GetAllPermutatedAngles(delta_theta);

  std::vector<std::vector<double>> pts_on_sphere = 
    GetPointsOnSphere(cov, permutated_angles, p_iso);

  // clone, assign and return AgentPtr vector
  std::vector<AgentPtr> agent_list;
  for (const auto& pt : pts_on_sphere) {
    AgentPtr new_agent(new Agent(*agent));

    // TODO: this needs to be adapted
    // TODO: if we have d_lon, d_lat, dv_lat, dv_lon
    // TODO: how to calculate agent state then?
    State state = agent->GetCurrentState();
    for (int i = 1; i <= pt.size(); i++) {
      state(i) += pt[i-1];
    }

    new_agent->SetCurrentState(state);
    agent_list.push_back(new_agent);
  }

  return agent_list;
}

}
}  // namespace observer
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_OBSERVER_UTILS_ELLIPSIS_HPP_
