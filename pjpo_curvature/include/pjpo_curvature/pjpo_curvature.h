/*
 * @Author: badaolaziang zhangziang@zju.edu.cn
 * @Date: 2022-10-01 17:13:41
 * @LastEditors: badaolaziang zhangziang@zju.edu.cn
 * @LastEditTime: 2022-11-24 19:55:56
 * @FilePath: /cartesian_plan/src/pjpo_curvature/include/pjpo_curvature/cartesian_planner.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include "cartesian_planner_config.h"

#include "dp_planner.h"
// #include "trajectory_optimizer.h"

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"
#include <string>
#include "pjpo_curvature/FrenetPath.h"

namespace pjpo_curvature {

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;


class PJPOCurvature {
public:
  struct StartState {
    double x, y, theta, v, phi, a, omega;
  };

  explicit PJPOCurvature(const CartesianPlannerConfig &config, const Env &env)
    : config_(config), dp_(config, env) {}

  bool Plan(const StartState &state, DiscretizedTrajectory &result);

  bool EvaluatePathKappa(std::string path_name, DiscretizedTrajectory & reference_path, 
                          const std::vector<double> & vec_x, const std::vector<double> & vec_y);

  double CalculateKappa(DiscretizedTrajectory & path, int idx);

  double CalculateKappa(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3);

  bool PJPOWithCurvature( const DiscretizedTrajectory & initial_path, int iteration);

  void VisualizeResult(int N, const Eigen::VectorXd & newQPSolution);

  bool UpdatePath(Eigen::VectorXd & newQPSolution, const DiscretizedTrajectory & reference_path, DiscretizedTrajectory & optimized_path);

  bool PubSLPath(const DiscretizedTrajectory & path, std::string path_id);

  // bool SpeedGenWithPath(const DiscretizedTrajectory & path, )

  std::vector<double> ds_1, ds_2, ds_3;
  vec_E<Eigen::Matrix3d> Avec_;
  vec_E<Eigen::Vector3d> Cvec_;
  vec_E<Eigen::Matrix3d> A_vec_;
  vec_E<Eigen::Vector3d> C_vec_;
  vec_E<Eigen::Matrix<double, 6, 6> > A_cross_vec_; //k_{i+1}*k_i项，顺序是l_0, dl_0, ddl_0, l_1, dl_1, ddl_1
  vec_E<Eigen::Matrix<double, 6, 1> > C_cross_vec_;

  //path smooth related parameters
  double smooth_bounding_dist, w_smooth;
  //weight of l, dl, ddl, dddl and kappa, dkappa
  double w_l, w_dl, w_ddl, w_dddl, w_obs, w_kappa_2, w_dkappa;
  //weight of speed, lateral acc, lateral jerk, kappa and dkappa limit.
  double lat_speed_limit, lat_acc_limit, lat_jerk_limit, kappa_limit, dkappa_limit;
  double starting_l ,car_info_L;
  int QPIteration_num = 0;

  double path_smoothing_duration, origin_qp_calculation_duration, new_qp_calculation_duration;
private:
  CartesianPlannerConfig config_;
  DpPlanner dp_;

  //path smoothing
  Eigen::MatrixXd smoothHessian;   //P of path smooth
  Eigen::SparseMatrix<double> smoothHessianSparse;  ////sparse type
  Eigen::VectorXd smoothGradient; //f of path smooth
  Eigen::MatrixXd smoothLinearMatrix;  //A
  Eigen::SparseMatrix<double> smoothLinearMatrixSparse; //sparse type
  Eigen::VectorXd smoothLowerBound; //low_limit
  Eigen::VectorXd smoothUpperBound; //upp_limit
  OsqpEigen::Solver pathSmoothSolver;
  Eigen::VectorXd pathSmoothQPSolution;

  // TrajectoryOptimizer opti_;
  Eigen::SparseMatrix<double> hessian_;  //P of origin PJPO
  Eigen::VectorXd gradientQ; //f of origin PJPO
  Eigen::MatrixXd hessianNew;   //P of proposed new method
  Eigen::VectorXd gradientQNew; //f of proposed new method
  Eigen::SparseMatrix<double> hessianNewSparse;  //sparse type

  Eigen::MatrixXd linearMatrix;  //A
  Eigen::SparseMatrix<double> linearMatrix_; //sparse type
  Eigen::VectorXd lowerBound; //low_limit
  Eigen::VectorXd upperBound; //upp_limit
  Eigen::VectorXd lMidRef; //mean value of lowerBound and upperBound
  OsqpEigen::Solver solver;
  OsqpEigen::Solver solverNew;
  OsqpEigen::Solver solver_t;
  Eigen::VectorXd newQPSolution;
  DiscretizedTrajectory smoothed_trajectory;
  DiscretizedTrajectory origin_qp_path;
  std::vector<DiscretizedTrajectory> optimized_path_vec;

  
};

}