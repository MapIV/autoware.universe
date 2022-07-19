// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "splines/interpolating_spline_pcg.hpp"
#include <algorithm>
#include <functional>
#include <vector>

namespace ns_splines
{
InterpolatingSplinePCG::InterpolatingSplinePCG(size_t interpolating_type)
  : interp_type_{interpolating_type}
{

}

bool InterpolatingSplinePCG::Interpolate(std::vector<double> const &tbase,
                                         std::vector<double> const &ybase,
                                         std::vector<double> const &tnew,
                                         std::vector<double> &ynew)
{

  bool couldSolve_and_Monotonic = prepareCoefficients(tbase, ybase);

  // Binary Search.
  for (auto const &ti : tnew)
  {
    if (ti < tbase[0] || ti > tbase.back())
    {
      double yval{};
      ns_utils::extrapolate(tbase, ybase, ti, yval);
      ynew.emplace_back(yval);
      continue;
    }
    auto const &left_ind = ns_utils::binary_index_search(ti, tbase);
    auto const &picewise_coeffs = coefficients_[left_ind];

    // Compute mpc_dt.
    auto const &dt = (ti - tbase[left_ind]) / (tbase[left_ind + 1] - tbase[left_ind]);

    //std::cout << mpc_dt << std::endl;
    auto const &yval = evaluatePolynomial(dt, picewise_coeffs);
    ynew.emplace_back(yval);
  }

  return couldSolve_and_Monotonic;
}

bool InterpolatingSplinePCG::Interpolate(std::vector<double> const &tnew,
                                         std::vector<double> &ynew) const
{

  if (!initialized_ || tbase_.empty() || ybase_.empty())
  {
    ns_utils::print("The interpolator is not initialized. Please initialize first.");
    return false;
  }

  // Binary Search.
  for (auto const &ti : tnew)
  {

    if (ti < tbase_[0] || ti > tbase_.back())
    {
      double yval{};
      ns_utils::extrapolate(tbase_, ybase_, ti, yval);
      ynew.emplace_back(yval);
      continue;
    }

    auto const &left_ind = ns_utils::binary_index_search(ti, tbase_);
    auto const &picewise_coeffs = coefficients_[left_ind];

    // Compute mpc_dt.
    auto const &dt = (ti - tbase_[left_ind]) / (tbase_[left_ind + 1] - tbase_[left_ind]);

    // std::cout << mpc_dt << std::endl;
    auto const &yval = evaluatePolynomial(dt, picewise_coeffs);
    ynew.emplace_back(yval);
  }

  return true;
}

// Point-wise scalar interpolation - std::vector case.
bool InterpolatingSplinePCG::Interpolate(const std::vector<double> &tbase,
                                         const std::vector<double> &ybase,
                                         const double &tnew,
                                         double &ynew)
{

  bool couldSolve_and_Monotonic = prepareCoefficients(tbase, ybase);

  if (!couldSolve_and_Monotonic)
  {
    return false;
  }

  if (tnew < tbase[0] || tnew > tbase.back())
  {
    ns_utils::extrapolate(tbase, ybase, tnew, ynew);
    return true;
  }

  // Binary Search.
  auto const &left_ind = ns_utils::binary_index_search(tnew, tbase);
  auto const &picewise_coeffs = coefficients_[left_ind];

  // Compute mpc_dt.
  auto const &dt = (tnew - tbase[left_ind]) / (tbase[left_ind + 1] - tbase[left_ind]);

  // Set the interpolated value.
  ynew = evaluatePolynomial(dt, picewise_coeffs);

  return couldSolve_and_Monotonic;
}

bool InterpolatingSplinePCG::Interpolate(const double &tnew, double &ynew) const
{

  if (tnew < tbase_[0] || tnew > tbase_.back())
  {
    ns_utils::extrapolate(tbase_, ybase_, tnew, ynew);
    return true;
  }


  // Binary Search.
  auto const &left_ind = ns_utils::binary_index_search(tnew, tbase_);
  auto const &picewise_coeffs = coefficients_[left_ind];

  // Compute mpc_dt.
  auto const &dt = (tnew - tbase_[left_ind]) / (tbase_[left_ind + 1] - tbase_[left_ind]);

  // Set the interpolated value.
  ynew = evaluatePolynomial(dt, picewise_coeffs);

  return true;
}

bool InterpolatingSplinePCG::compute_coefficients(std::vector<double> const &ybase)
{

  auto dimy(static_cast<int>(ybase.size()));
  Eigen::Index dimy_eig(dimy);

  // Jacobi Preconditioner Iteration.
  std::vector<double> cmat_diag;

  if (interp_type_ == 1)
  {
    // No need to solve equations. Equations is y = ax + b_coeff, te coeeficients are [a, b_coeff], bi = (anext - aprev)
    std::vector<double> b_coeff;

    // compute differences (a_{i+1} - a_i)
    std::transform(ybase.cbegin(), ybase.cend(), ybase.cbegin() + 1, std::back_inserter(b_coeff),
                   [](auto &v0, auto &v1)
                   { return v1 - v0; });

    b_coeff.emplace_back(0.0); // b_coeff[n] = 0

    // Push [ai, bi] into the coefficients_ vector of vectors.
    std::transform(ybase.cbegin(), ybase.cend(), b_coeff.cbegin(), std::back_inserter(coefficients_),
                   [](auto &ai, auto &bi)
                   {
                     return std::vector<double>{ai, bi};
                   });
    return true;

  }
  if (interp_type_ == 3)
  {

    // First prepare cmat diagonals for Jacobi iterations. And pass it to PCG
    /*
     *  Ax = b_rhs
     *  diag_lower = diag(A, k=-1) upper diagonal
     *  diag       = diag(A, k=0)
     *  diag_upper = diag(A, k=1) upper diagonal
     * */

    // Create a Sparse Matrix Triplets

    std::vector<Eigen::Triplet<double>> tripletVec;

    for (int k = 1; k < dimy - 1; k++)
    {
      tripletVec.emplace_back(Eigen::Triplet<double>{k, k - 1, 1}); // set lower diagonal values
      tripletVec.emplace_back(Eigen::Triplet<double>{k, k, 4.}); // set diagonal values
      tripletVec.emplace_back(Eigen::Triplet<double>{k, k + 1, 1.}); // set upper diagonals
    }

    // Set terminals of Asparse matrix.
    tripletVec.emplace_back(Eigen::Triplet<double>{0, 0, 1.0});
    tripletVec.emplace_back(Eigen::Triplet<double>{dimy - 1, dimy - 1, 1.0});

    // Create a sparse matrix of A.
    Eigen::SparseMatrix<double> Asprs(dimy_eig, dimy_eig);
    Asprs.setFromTriplets(tripletVec.cbegin(), tripletVec.cend());

    // Prepare right handside b.
    std::vector<double> brhs(dimy, 0.0); // right hand side of Ax=b.

    // Compute rhs values;  b[2:N-1]_i = 3*(a(i) - 2*a_(i) + a(i+1)
    for (size_t k = 1; k < dimy - 1; k++)
    {
      brhs[k] = 3 * (ybase[k - 1] - 2 * ybase[k] + ybase[k + 1]);
    }

    Eigen::Map<Eigen::VectorXd> b(brhs.data(), dimy_eig, 1);

    //    // DEBUG
    //    ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Asprs));
    //
    //    auto main_diag = Asprs.diagonal();
    //    ns_eigen_utils::printEigenMat(main_diag);
    //    ns_eigen_utils::printEigenMat(b);
    //    // End of DEBUG

    std::vector<double> solution_c(dimy);
    // Solve Ax = b.

    if (bool isSolved = pcg_.solve(Asprs, b, solution_c);isSolved)
    {
      // Set Coefficients
      set_bd_coeffs_(ybase, solution_c);
      initialized_ = true; // set for the further use if coefficients are initialized.
      return true;

    }
    ns_utils::print("Failed to solve for interpolating coefficients .. \n");
    return false;

  }
  ns_utils::print("\nOnly polynomial degrees one and three are implemented ... \n ");
  return false;

}

void InterpolatingSplinePCG::set_bd_coeffs_(std::vector<double> const &ybase,
                                            std::vector<double> const &c_coeffs)
{
  // Set d_coeffs = (c(i+1) - c(i)) / 3.
  // set bi = a1 - a0 - (c1 + 2*c0)/3;

  size_t dimc = c_coeffs.size();
  std::vector<double> b_coeffs;
  std::vector<double> d_coeffs;

  // Set coeff d.
  std::transform(c_coeffs.cbegin(), c_coeffs.cend(), c_coeffs.begin() + 1, std::back_inserter(d_coeffs),
                 [](auto c0, auto c1)
                 {
                   return (c1 - c0) / 3;
                 });

  d_coeffs.emplace_back(0.0); // set last d[n]=0

  // Set coeff b.
  auto &a_coeff = ybase;

  for (size_t k = 0; k < dimc - 1; k++) // set coeffs[0:n-1]
  {
    auto &&b = (a_coeff[k + 1] - a_coeff[k]) - (c_coeffs[k + 1] + 2 * c_coeffs[k]) / 3;
    b_coeffs.emplace_back(b);

    coefficients_.emplace_back(std::vector<double>{a_coeff[k], b_coeffs[k], c_coeffs[k], d_coeffs[k]});

  }

  b_coeffs.emplace_back(0.0);
  auto kf = dimc - 1;

  // Set coeffs[n]
  coefficients_.emplace_back(std::vector<double>{a_coeff[kf], b_coeffs[kf], c_coeffs[kf], d_coeffs[kf]});

}

bool InterpolatingSplinePCG::prepareCoefficients(std::vector<double> const &tbase,
                                                 std::vector<double> const &ybase)
{
  // Just produces a warning.
  if (bool isMonotonic = checkIfMonotonic(tbase); !isMonotonic)
  {
    return false; // Data is not a monotonic series.
  }

  // Reset previously computed values.
  coefficients_.clear();

  if (auto const couldSolve = compute_coefficients(ybase);!couldSolve)
  {
    ns_utils::print("[spline_pcg]: Couldn't solve the problem ...\n");
    return false; // couldn't solve.
  }

  return true;
}

bool InterpolatingSplinePCG::checkIfMonotonic(const std::vector<double> &tbase)
{
  // Check if interpolating coordinates is monotonic.
  auto isIncreasing_it = std::adjacent_find(tbase.begin(), tbase.end(), std::greater_equal<>());
  auto isDecreasing_it = std::adjacent_find(tbase.begin(), tbase.end(), std::less_equal<>());

  bool isMonotonic = !(isDecreasing_it == tbase.cend() && isIncreasing_it == tbase.cend());

  if (!isMonotonic)
  {
    ns_utils::print("\n Data coordinates are not monotonic series ... \n");
  }

  return isMonotonic;
}

/*
 *  When we call interpolate method, interpolator checks whether there are existing parameters to use. If we are not
 *  using the existing parameters we need to initialize the interpolator setting reusing_coefficients=false, so that
 *  the interpolator re-computes the interpolation coefficients.
 * */
bool InterpolatingSplinePCG::Initialize(std::vector<double> const &tbase, std::vector<double> const &ybase)
{
  double t0 = tbase[0];
  double y0{};
  bool couldInterpolate = Interpolate(tbase, ybase, t0, y0);

  if (couldInterpolate)
  {
    tbase_ = tbase;
    ybase_ = ybase;
  }

  return couldInterpolate;
}

double InterpolatingSplinePCG::evaluatePolynomial(double ti, std::vector<double> coeffs)
{
  size_t nitems = coeffs.size();
  double sum_ttimes_coeff = 0.0;

  for (size_t k = 0; k < nitems; k++)
  {
    sum_ttimes_coeff += std::pow<double>(ti, k) * coeffs[k];
  }

  return sum_ttimes_coeff;
}

bool PCG::solve(Eigen::SparseMatrix<double> const &Asparse,
                Eigen::VectorXd const &b, std::vector<double> &solution_c) const
{

  // Initialize a solution vector xn with zero.
  Eigen::VectorXd xn(b.rows(), 1); // solution to Ax = b, xn is the coefficients of a polynomial
  xn.setZero(); // value of x at the nth iterations.

  // Initialize residuals.
  Eigen::VectorXd rn(b); // residuals r(n) n is the iteration number.

  //  ns_eigen_utils::printEigenMat(rn);
  // Initialize inverse of the diagonals.
  Eigen::VectorXd invDiag(b.rows(), 1);
  invDiag << Asparse.diagonal().unaryExpr([](auto x)
                                          { return 1 / x; });


  // Solve for rtilda.
  Eigen::VectorXd rtilda_n(invDiag.cwiseProduct(rn));

  // Initialize search direction pn.
  auto pn = rtilda_n;

  //  ns_eigen_utils::printEigenMat(rtilda_n);
  // Compute the step length.
  auto alpha_n = rn.dot(rtilda_n) / pApnorm(pn);

  // Update xn
  xn = xn + alpha_n * pn;

  // Set (n-1)th residuals to use in the recursive solution.
  auto rn_prev = rn;

  // Update the residuals.
  //  auto &&Ap = pAdot(pn);
  rn = rn - alpha_n * Asparse * pn;
  //  rn = rn - alpha_n * pAdot(pn);

//  ns_eigen_utils::printEigenMat(rn);
  // Define gradient correction factor beta.
  double beta{};

  // DEBUG
  // std::cout << "\nrn_tilda \n";
  // ns_eigen_utils::printEigenMat(rtilda_n);
  //  ns_eigen_utils::printEigenMat(invDiag);
  //end of DEBUG

  for (size_t k = 0; k < maxiter_; k++)
  {
    auto rtilda_prev = rtilda_n; // copy
    rtilda_n = invDiag.cwiseProduct(rn);

    // Compute gradient direction correction factor.
    beta = rn.dot(rtilda_n) / rn_prev.dot(rtilda_prev);

    // Update the search direction.
    pn.noalias() = rtilda_n + beta * pn;

    // Compute alpha; step length.
//    alpha_n = rn.dot(rtilda_n) / pApnorm(pn);
    alpha_n = rn.dot(rtilda_n) / pn.dot(Asparse * pn);

    // Update the solution.
    xn = xn + alpha_n * pn;

    // Update r(n-1) and r(n).
    rn_prev = rn;
    rn.noalias() = rn - alpha_n * Asparse * pn;

    //    rn.noalias() = rn - alpha_n * pAdot(pn);
    //    std::cout << rn.rows() << " " << pAdot(pn).rows() << std::endl;
    //    ns_eigen_utils::printEigenMat(pAdot(pn) - rn);

    if (isConvergedL1(rn))
    {
      // Set solution and return.
      Eigen::VectorXd::Map(&solution_c[0], xn.size()) = xn;
      return true;
    }
  }

  return false;
}

bool PCG::isConvergedL1(Eigen::VectorXd const &residuals) const
{
  auto l1norm = residuals.cwiseAbs().sum();
//  std::cout << "L1 norm of the residuals is : " << l1norm << std::endl;
  return l1norm < eps_;
}

double PCG::pApnorm(const Eigen::MatrixXd &p)
{
  auto dim_p = p.rows();

  // results, p[0]**2 + p[n]**2
  double panorm = p(0) * p(0) + p(dim_p - 1) * p(dim_p - 1);
  for (int k = 1; k < (int) dim_p - 1; k++)
  {
    panorm += p(k) * (p(k - 1) + 4 * p(k) + p(k + 1));
  }

  return panorm;
}

} // namespace ns_splines