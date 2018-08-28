// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.
#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include "ceres/ceres.h"

struct ReProjectionError {
  ReProjectionError(double observed_x, double observed_y, const double *point)
      : observed_x(observed_x), observed_y(observed_y)
      {
        point_[0] = point[0];
        point_[1] = point[1];
        point_[2] = point[2];
      }
  template <typename T>
  bool operator()(const T* const camera, T* residuals)
  const {

    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double *point) {
    return (new ceres::AutoDiffCostFunction<ReProjectionError, 2, 8>(
                new ReProjectionError(observed_x, observed_y, point)));
  }
  double observed_x;
  double observed_y;
  double point_[3];
};
int main(int argc, char** argv) {
  ceres::Problem problem;

  cv::FileStorage fs2("src/camera_calibration_virtana/calibration_points2.xml", cv::FileStorage::READ);

  cv::FileNode points = fs2["points"];

  std::vector<std::vector<float>> xyz;

  std::vector<std::vector<float>> pixels;

  for(cv::FileNodeIterator fi = points.begin(); fi != points.end(); fi++)
  {
    for(int i = 0; i < ((*fi)["corners"]).size(); i++)
    {
      std::vector<float> temp1;
      temp1.push_back((*fi)["corners"][i]["transform"]["X"]);
      temp1.push_back((*fi)["corners"][i]["transform"]["Y"]);
      temp1.push_back((*fi)["corners"][i]["transform"]["Z"]);
      xyz.push_back(temp1);

      std::vector<float> temp2;
      temp1.push_back((*fi)["corners"][i]["pixel"]["x"]);
      temp1.push_back((*fi)["corners"][i]["pixel"]["y"]);

      pixels.push_back(temp2);
    }
  }

//  for (int i = 0; i < 10; ++i) {
//    // Each Residual block takes a point and a camera as input and outputs a 2
//    // dimensional residual. Internally, the cost function stores the observed
//    // image location and compares the reprojection against the observation.
////    ceres::CostFunction* cost_function =
////        ReProjectionError::Create(observations[2 * i + 0],
////                                         observations[2 * i + 1]);
////    problem.AddResidualBlock(cost_function,
////                             NULL /* squared loss */,
////                             bal_problem.mutable_camera_for_observation(i),
////                             bal_problem.mutable_point_for_observation(i));
//  }
//  // Make Ceres automatically detect the bundle structure. Note that the
//  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
//  // for standard bundle adjustment problems.
//  ceres::Solver::Options options;
//  options.linear_solver_type = ceres::DENSE_SCHUR;
//  options.minimizer_progress_to_stdout = true;
//  ceres::Solver::Summary summary;
//  ceres::Solve(options, &problem, &summary);
//  std::cout << summary.FullReport() << "\n";
  return 0;
}
