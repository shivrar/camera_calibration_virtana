#include "ceres/ceres.h"
#include "glog/logging.h"

#include <time.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>


using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace cv;
using namespace std;

//struct CalFunctor{
//    template <typename T> bool operator()(const T* const x, T* x, T* y) const {
////    residual[0] = 10.0 - x[0];
//
////    sum of (expected - actual)^2 for all matched points
//
//    return true;
//  }
//}

// A templated cost functor that implements the residual r = 10 -
// x. The method operator() is templated so that we can then use an
// automatic differentiation wrapper around it to generate its
// derivatives.
struct CostFunctor {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, NULL, &x);

  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";


//load the data

//  FileStorage fs2("src/camera_calibration_virtana/calibration_points.xml", FileStorage::READ);
//
//  FileNode points = fs2["points"];
//
//
//  for(FileNodeIterator it = points.begin(); it != points.end(); it++)
//  {
////    ROS_INFO("%i",(int)((((*it)["corners"])[0])["pixel"])["x"]);
//  }
//
//  fs2.release();


  return 0;
}