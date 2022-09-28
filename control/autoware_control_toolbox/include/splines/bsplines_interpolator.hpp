// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_INTERPOLATOR_HPP_
#define AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_INTERPOLATOR_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/IterativeLinearSolvers>
#include <eigen3/Eigen/Sparse>
#include <algorithm>
#include <functional>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

namespace ns_splines
{
class BSplineInterpolator
{
public:
  BSplineInterpolator()
  : n_base_points_{2},
    new_npoints_{2},
    knots_ratio_{0.3}, compute_derivatives_{true}, nknots_{1}
  {

  }

  explicit BSplineInterpolator(
    size_t base_signal_length,
    size_t new_length,
    double num_of_knots_ratio = 0.3,
    bool compute_derivatives = false);

  explicit BSplineInterpolator(
    Eigen::MatrixXd const & tvec_base,
    Eigen::MatrixXd const & tvec_new,
    double num_of_knots_ratio = 0.3,
    bool compute_derivatives = false);

  // Copy Constructors
  BSplineInterpolator(BSplineInterpolator const & other)
  : n_base_points_{other.n_base_points_},
    new_npoints_{other.new_npoints_},
    knots_ratio_{other.knots_ratio_},
    compute_derivatives_{other.compute_derivatives_},
    nknots_{other.nknots_},
    projection_mat_base_{other.projection_mat_base_},
    projection_mat_w_new_base_{other.projection_mat_w_new_base_},
    projection_mat_w_new_base_dot_{
      other.projection_mat_w_new_base_dot_},
    projection_mat_w_new_base_dot_dot_{
      other.projection_mat_w_new_base_dot_dot_}
  {

  }

  BSplineInterpolator & operator=(BSplineInterpolator const & other)
  {
    if (&other != this) {
      n_base_points_ = other.n_base_points_;
      new_npoints_ = other.new_npoints_;
      knots_ratio_ = other.knots_ratio_;
      compute_derivatives_ = other.compute_derivatives_;
      nknots_ = other.nknots_;

      projection_mat_base_ = other.projection_mat_base_;
      projection_mat_w_new_base_ = other.projection_mat_w_new_base_;
      projection_mat_w_new_base_dot_ = other.projection_mat_w_new_base_dot_;
      projection_mat_w_new_base_dot_dot_ = other.projection_mat_w_new_base_dot_dot_;
    }

    return *this;
  }

  // Move constructors.
  BSplineInterpolator(BSplineInterpolator && other) noexcept
  : n_base_points_{other.n_base_points_},
    new_npoints_{other.new_npoints_},
    knots_ratio_{other.knots_ratio_},
    compute_derivatives_{other.compute_derivatives_},
    nknots_{other.nknots_},
    projection_mat_base_{std::move(other.projection_mat_base_)},
    projection_mat_w_new_base_{std::move(other.projection_mat_w_new_base_)},
    projection_mat_w_new_base_dot_{std::move(other.projection_mat_w_new_base_dot_)},
    projection_mat_w_new_base_dot_dot_{std::move(other.projection_mat_w_new_base_dot_dot_)}
  {

  }

  BSplineInterpolator & operator=(BSplineInterpolator && other) noexcept
  {
    if (&other != this) {
      n_base_points_ = other.n_base_points_;
      new_npoints_ = other.new_npoints_;
      knots_ratio_ = other.knots_ratio_;
      compute_derivatives_ = other.compute_derivatives_;
      nknots_ = other.nknots_;

      projection_mat_base_ = std::move(other.projection_mat_base_);
      projection_mat_w_new_base_ = std::move(other.projection_mat_w_new_base_);
      projection_mat_w_new_base_dot_ = std::move(other.projection_mat_w_new_base_dot_);
      projection_mat_w_new_base_dot_dot_ = std::move(other.projection_mat_w_new_base_dot_dot_);
    }
    return *this;
  }

  // Each column of base matrix is used to find new interpolated matrix column with
  // a different size.
  void InterpolateInCoordinates(
    Eigen::MatrixXd const & ybase, Eigen::MatrixXd & data_tobe_interpolated);

  // If the derivatives are requested, one can call these two following methods.
  // Base data is provided, first and second derivatives are returned back as the second terms.
  void getFirstDerivative(Eigen::MatrixXd const & ybase, Eigen::MatrixXd & ybase_dot);

  void getSecondDerivative(Eigen::MatrixXd const & ybase, Eigen::MatrixXd & ybase_dot_dot);

private:
  // Pre-settings. Increasing lambda yield more smooth and flattened curve.

  // smoothing factor used in normal form of LS; B*B + (lambda**2)*D*D, D is f''(x).
  double lambda_ = 0.001;

  // Initialized during instantiation.
  size_t n_base_points_{};  // number of points in the base,
  size_t new_npoints_{};    // number of points in the new dimensions
  double knots_ratio_{};    // ratio of the number of knots to number of interpolation points;
  bool compute_derivatives_{};
  size_t nknots_{};

  std::vector<double> knots_vec_;  // knot points of the global smoother.

  // To be computed in the constructor.
  /*
   *   A = A(t) parametric polynomials in the rows, each row has [1, t, t**2, t**3, (t-knot)**3_i ... ]
   *   y = Ax --> A^T*y = A^T A*x -- > x = coeffs = inv(A^T*A)*A*ybase_data
   *                          -- > ynew = Anew_base(t) * x  = Anew(t) * coeffs  = Anew(t) * inv(A^T*A)*A * ydata
   *                          -- > projection matrix p = inv(A^T*A)*A
   *                          ---> projection with base included  Anew(t) * inv(At*A)*A
   *
   * */
  Eigen::MatrixXd projection_mat_base_;  // Base signal to be interpolated from.

  // inv(A^T*A)*A  when ydata comes it becomes coeffs = x = inv(A^T*A)*A * ybase_data
  Eigen::MatrixXd projection_mat_w_new_base_;

  // ysmooth_dot = basis_dot * projection_mat_dot*ydata
  Eigen::MatrixXd projection_mat_w_new_base_dot_;

  // ysmooth_ddot = basis_ddot * projection_mat_ddot*ydata - second derivative
  Eigen::MatrixXd projection_mat_w_new_base_dot_dot_;

  // Inner Methods.
  // If the derivatives are not requested.
  void createBasesMatrix(Eigen::VectorXd const & tvec, Eigen::MatrixXd & basis_mat);

  // If the derivatives of the new interpolated data are requested.
  void createBasesMatrix(
    Eigen::VectorXd const & tvec, Eigen::MatrixXd & basis_mat,
    Eigen::MatrixXd & regularization_mat_dd);

  // Used to compute new interpolated data and its derivatives.
  void createBasesMatrix(
    Eigen::VectorXd const & tvec, Eigen::MatrixXd & basis_mat, Eigen::MatrixXd & basis_dmat,
    Eigen::MatrixXd & basis_ddmat);

  void solveByDemmlerReisch(
    Eigen::MatrixXd const & basis_mat,
    Eigen::MatrixXd const & penalizing_mat_D);                         // set projection mat for the base data.

  void solveByQR(Eigen::MatrixXd const & basis_mat, Eigen::MatrixXd const & penalizing_mat_D);
};
}  // namespace ns_splines

#endif //AUTOWARE_CONTROL_TOOLBOX_INCLUDE_SPLINES_BSPLINES_INTERPOLATOR_HPP_
