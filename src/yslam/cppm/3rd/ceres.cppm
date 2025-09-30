module;
#include <ceres/ceres.h>
#include <ceres/rotation.h>
export module ceres;
export namespace ceres {
using ceres::AngleAxisRotatePoint;
using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::EuclideanManifold;
using ceres::HuberLoss;
using ceres::LinearSolverType;
using ceres::LossFunction;
using ceres::Manifold;
using ceres::NumericDiffCostFunction;
using ceres::NumericDiffMethodType;
using ceres::PreconditionerType;
using ceres::Problem;
using ceres::ProductManifold;
using ceres::QuaternionManifold;
using ceres::QuaternionRotatePoint;
using ceres::Solve;
using ceres::Solver;
using ceres::SubsetManifold;

}