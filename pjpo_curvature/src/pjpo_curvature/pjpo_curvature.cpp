#include "pjpo_curvature/pjpo_curvature.h"
#include "pjpo_curvature/visualization/plot.h"
#include "pjpo_curvature/tic_toc.h"
#include "geometry_msgs/PoseArray.h" 

namespace pjpo_curvature {

bool PJPOCurvature::Plan(const StartState &state, DiscretizedTrajectory &result) {
  DiscretizedTrajectory coarse_trajectory = dp_.env_->reference();
  std::cout << "The size of coarse_trajectory is " << coarse_trajectory.data_.size() <<   std::endl;
  

  std::vector<double> coarse_x, coarse_y;
  for(auto &pt: coarse_trajectory.data()) {
    coarse_x.push_back(pt.x); coarse_y.push_back(pt.y);
  }

  visualization::Plot(coarse_x, coarse_y, 0.03, visualization::Color::Black, 1, "Coarse Trajectory");
  // visualization::PlotPoints(coarse_x, coarse_y, 0.2, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();

  int N = coarse_trajectory.data_.size();

  if(N < 5){
    //point number too small
    ROS_WARN("Have no enough points.");
    return false;
  }

  //path smoothing
  smoothed_trajectory = coarse_trajectory;
  int smooth_decision_variable_num = N * 2; // (x, y) * N
  int smooth_constraint_num = N * 2; //(x, y) * the upper and lower constraint of N reference points
  smoothHessian.resize(smooth_decision_variable_num, smooth_decision_variable_num);
  smoothHessian.setZero();
  smoothGradient.resize(smooth_decision_variable_num);
  smoothGradient.setZero();
  smoothLinearMatrix.resize(smooth_constraint_num, smooth_constraint_num);
  smoothLinearMatrix.setZero();
  smoothLowerBound.resize(smooth_constraint_num);
  smoothUpperBound.resize(smooth_constraint_num);
  //adding the boundingbox constraint
  smoothLinearMatrix(0, 0) = 1;
  smoothLowerBound[0] = smoothed_trajectory.data_[0].x;
  smoothUpperBound[0] = smoothed_trajectory.data_[0].x;
  smoothLinearMatrix(N, N) = 1;
  smoothLowerBound[N] = smoothed_trajectory.data_[0].y;
  smoothUpperBound[N] = smoothed_trajectory.data_[0].y;
  for(int i = 1; i < N -1 ; ++i){
    smoothLinearMatrix(i, i) = 1;
    smoothLowerBound[i] = smoothed_trajectory.data_[i].x - smooth_bounding_dist;
    smoothUpperBound[i] = smoothed_trajectory.data_[i].x + smooth_bounding_dist;
    smoothLinearMatrix(i+N, i+N) = 1;
    smoothLowerBound[i + N] = smoothed_trajectory.data_[i].y - smooth_bounding_dist;
    smoothUpperBound[i + N] = smoothed_trajectory.data_[i].y + smooth_bounding_dist;
  }
  smoothLinearMatrix(N-1, N-1) = 1;
  smoothLowerBound[N - 1] = smoothed_trajectory.data_[N-1].x;
  smoothUpperBound[N - 1] = smoothed_trajectory.data_[N-1].x;
  smoothLinearMatrix(N*2-1, N*2-1) = 1;
  smoothLowerBound[N*2 - 1] = smoothed_trajectory.data_[N-1].y;
  smoothUpperBound[N*2 - 1] = smoothed_trajectory.data_[N-1].y;

  //adding smooth hessian matrix
  for(int i = 1; i < N - 1; ++i){
    smoothHessian(i-1, i-1) += w_smooth;
    smoothHessian(i, i) += 4.0 * w_smooth;
    smoothHessian(i+1, i+1) += w_smooth;
    smoothHessian(i-1, i) += -2.0*w_smooth;
    smoothHessian(i, i-1) += -2.0*w_smooth;
    smoothHessian(i-1, i+1) += w_smooth;
    smoothHessian(i+1, i-1) += w_smooth;
    smoothHessian(i, i+1) += -2.0*w_smooth;
    smoothHessian(i+1, i) += -2.0*w_smooth;
    
    smoothHessian(N+i-1, N+i-1) += w_smooth;
    smoothHessian(N+i, N+i) += 4.0 * w_smooth;
    smoothHessian(N+i+1, N+i+1) += w_smooth;
    smoothHessian(N+i-1, N+i) += -2.0*w_smooth;
    smoothHessian(N+i, N+i+1) += -2.0*w_smooth;
    smoothHessian(N+i-1, N+i+1) += w_smooth;
    smoothHessian(N+i, N+i-1) += -2.0*w_smooth;
    smoothHessian(N+i+1, N+i) += -2.0*w_smooth;
    smoothHessian(N+i+1, N+i-1) += w_smooth;
  }
  smoothHessianSparse = smoothHessian.sparseView();
  smoothLinearMatrixSparse = smoothLinearMatrix.sparseView();

  // settings
  pathSmoothSolver.settings()->setVerbosity(false);
  pathSmoothSolver.settings()->setWarmStart(true);
  // pathSmoothSolver.settings()->setAbsoluteTolerance(1e-6);
  // pathSmoothSolver.settings()->setMaxIteration(30000);
  // pathSmoothSolver.settings()->setRelativeTolerance(1e-6);

  // set the initial data of the QP solver
  pathSmoothSolver.data()->setNumberOfVariables(smooth_decision_variable_num); //set the colum number of matrix A, which is n
  pathSmoothSolver.data()->setNumberOfConstraints(smooth_constraint_num); //set the row number of matrix A, which is m
  pathSmoothSolver.data()->clearHessianMatrix();
  pathSmoothSolver.data()->clearLinearConstraintsMatrix();
  if(!pathSmoothSolver.data()->setHessianMatrix(smoothHessianSparse)) return 1;//set matrix P
  if(!pathSmoothSolver.data()->setGradient(smoothGradient)) return 1; //set the matrix q (or f)
  if(!pathSmoothSolver.data()->setLinearConstraintsMatrix(smoothLinearMatrixSparse)) return 1;//set the linear cosntraint matrix
  if(!pathSmoothSolver.data()->setLowerBound(smoothLowerBound)) return 1;//set the lower bound
  if(!pathSmoothSolver.data()->setUpperBound(smoothUpperBound)) return 1;//set the upper bound

  // instantiate the solver
  if(pathSmoothSolver.isInitialized()){
    pathSmoothSolver.clearSolver();
  }
  if(!pathSmoothSolver.initSolver()) return 1;

  TicToc path_smoothing_timer;
  path_smoothing_timer.tic();

  // solve the QP problem
  if(pathSmoothSolver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << "Something wrong during the path smooth OSQP problem solving!" << std::endl;
    return 1;
  }

  // get the controller input
  pathSmoothQPSolution = pathSmoothSolver.getSolution();
  // pathSmoothSolver.clearSolver();
  path_smoothing_duration = path_smoothing_timer.toc();
  std::cout << "path smoothing duration is " << path_smoothing_duration << " ms" << std::endl;

  for(int i = 0; i < N; ++i){
    //reference https://www.matongxue.com/madocs/2106/
    smoothed_trajectory.data_[i].x = pathSmoothQPSolution[i];
    smoothed_trajectory.data_[i].y = pathSmoothQPSolution[i+N];
  }
  //deleting the front and back point
  smoothed_trajectory.data_.erase(smoothed_trajectory.data_.begin());
  smoothed_trajectory.data_.pop_back();
  N = N-2;
  for(int i = 1; i < N-1; ++i){
    //reference https://www.matongxue.com/madocs/2106/
    Eigen::Vector2d p1, p2, p3, v1, v2, v3;
    p1 << smoothed_trajectory.data_[i-1].x, smoothed_trajectory.data_[i-1].y;
    p2 << smoothed_trajectory.data_[i].x,   smoothed_trajectory.data_[i].y;
    p3 << smoothed_trajectory.data_[i+1].x, smoothed_trajectory.data_[i+1].y;
    smoothed_trajectory.data_[i].kappa = CalculateKappa(p1, p2, p3);
  }
  std::vector<double> smoothed_x, smoothed_y;
  for(int i = 0; i < N; ++i) {
    smoothed_x.push_back(smoothed_trajectory.data_[i].x);
    smoothed_y.push_back(smoothed_trajectory.data_[i].y);
  }
  visualization::Plot(smoothed_x, smoothed_y, 0.06, visualization::Color::Green, 1, "Smoothed Trajectory");
  // visualization::PlotPoints(coarse_x, coarse_y, 0.2, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();

  EvaluatePathKappa("SmoothedReference", smoothed_trajectory, smoothed_x, smoothed_y);
  
  ds_1.resize(N);
  ds_2.resize(N);
  ds_3.resize(N);
  // update the smoothed_trajectory
  for(int i = 0; i <smoothed_trajectory.data_.size() - 1; ++i){
    ds_1[i] = smoothed_trajectory.data_[i+1].s - smoothed_trajectory.data_[i].s; 
    // std:: cout << ds_1[i] << std::endl;
    smoothed_trajectory.data_[i].ds = ds_1[i];
    // std::cout << smoothed_trajectory.data_[i].ds << " " 
    //           << hypot(smoothed_trajectory.data_[i+1].x -smoothed_trajectory.data_[i].x, 
    //               smoothed_trajectory.data_[i+1].y -smoothed_trajectory.data_[i].y) << std::endl;
    // smoothed_trajectory.data_[i].left_bound  =  3.0;
    smoothed_trajectory.data_[i].right_bound = -smoothed_trajectory.data_[i].right_bound;
    smoothed_trajectory.data_[i].dkappa = (smoothed_trajectory.data_[i + 1].kappa - smoothed_trajectory.data_[i].kappa)/ds_1[i];
    smoothed_trajectory.data_[i].theta = std::atan2(smoothed_trajectory.data_[i+1].y - smoothed_trajectory.data_[i].y,
                                                    smoothed_trajectory.data_[i+1].x - smoothed_trajectory.data_[i].x);
  }
  ds_1[N - 1] = ds_1[N - 2];
  smoothed_trajectory.data_[N - 1].ds = ds_1[N - 1];
  for(int i = 0; i < N; ++i){
    ds_2[i] = pow(ds_1[i], 2);
    ds_3[i] = pow(ds_1[i], 3);
  }
  smoothed_trajectory.data_.back().right_bound = -smoothed_trajectory.data_.back().right_bound;
  PubSLPath(smoothed_trajectory,"SmoothedReference");

  //stage I: original PiecewiseJerk Path Optimization
  // here to add the PiecewiseJerkPathOptimization part
  int decision_variable_num = N * 3; // (l, dl, ddl) * N
  int constraint_num = 0;
  constraint_num += N*3; // the bound constraint of each l
  constraint_num += N - 1; // equation constraint between l_i+1 and l_i
  constraint_num += N - 1; // equation constraint between l'_i+1 and l'_i
  constraint_num += N - 1; // equation constraint between l''_i+1 and l''_i
  constraint_num += 3; // equation constraint of the starting point
  // The linear curvature constraint  
  constraint_num += N ;// the inequation constraint of the curvature of each point(the method from Zhang Yajia)
  constraint_num += N ;// the inequation constraint of the curvature of each point(the estimation of the 2nd-order Taylor estimation)
  // constraint_num += N-1 ;// the inequation constraint of the dkappa of each point
  // set the initial data of the QP solver
  
  linearMatrix.resize(constraint_num, decision_variable_num);
  linearMatrix.setZero();
  lowerBound.resize(constraint_num);
  lowerBound.setZero(constraint_num);
  upperBound.resize(constraint_num);
  upperBound.setZero(constraint_num);
  lMidRef.resize(constraint_num);
  lMidRef.setZero(constraint_num);

  // static obstacles

  // for(int i = 0; i < N; ++i){
  //   if( 15 <= smoothed_trajectory.data_[i].s && smoothed_trajectory.data_[i].s <= 20){
  //     smoothed_trajectory.data_[i].left_bound = -0.5;
  //   }
  //   if( 45 <= smoothed_trajectory.data_[i].s && smoothed_trajectory.data_[i].s <= 50){
  //     smoothed_trajectory.data_[i].right_bound = 0.5;
  //   }
  //   if( 75 <= smoothed_trajectory.data_[i].s && smoothed_trajectory.data_[i].s <= 80){
  //     smoothed_trajectory.data_[i].left_bound = -0.5;
  //   }
  //   if( 105 <= smoothed_trajectory.data_[i].s && smoothed_trajectory.data_[i].s <= 110){
  //     smoothed_trajectory.data_[i].right_bound = 0.5;
  //   }
  //   if( 130 <= smoothed_trajectory.data_[i].s && smoothed_trajectory.data_[i].s <= 140){
  //     smoothed_trajectory.data_[i].left_bound = -0.5;
  //   }
  // }

  //adding the lower and upper bound of each l
  for(int i = 0; i < N; ++i){
    linearMatrix(i, i) = 1.0;
    // lowerBound[i] = smoothed_trajectory.data_[i].right_bound + 1;
    // upperBound[i] = smoothed_trajectory.data_[i].left_bound - 1;

    lowerBound[i] = smoothed_trajectory.data_[i].right_bound;
    upperBound[i] = smoothed_trajectory.data_[i].left_bound;
    lMidRef[i] = (lowerBound[i] + upperBound[i])/2.0; //get the l_min_reference
  }


  hessian_.resize(decision_variable_num, decision_variable_num);
  hessian_.setZero();  
  gradientQ.resize(decision_variable_num);
  gradientQ.setZero();

  //adding the lower and upper bound of each l'
  for(int i = N; i < N*2; ++i){
    linearMatrix(i, i) = 1.0;
    lowerBound[i] = -lat_speed_limit;
    upperBound[i] =  lat_speed_limit;
  }
  //adding the lower and upper bound of each l''
  for(int i = 2*N; i < N*3; ++i){
    linearMatrix(i, i) = 1.0;
    lowerBound[i] = -lat_acc_limit;
    upperBound[i] =  lat_acc_limit;
  }

  //adding the equation constraint between l_i+1 and l_i
  for(int i = 0; i < N - 1; ++i){
    // l_i+1 - l_i - l'_i*delta_s - 1/2*l''_i*delta_s^2 = 1/6 *jerk * delta_s^3
    linearMatrix(i + N*3, i + 1)     =   1.0; 
    linearMatrix(i + N*3, i    )     =  -1.0; 
    linearMatrix(i + N*3, i + N)     =  -ds_1[i];
    linearMatrix(i + N*3, i + N*2)   =  -ds_2[i]/2.0;
    lowerBound[i + N*3] = -1.0/6.0*lat_jerk_limit*ds_3[i];
    upperBound[i + N*3] =  1.0/6.0*lat_jerk_limit*ds_3[i];
  }

  //adding the equation constraint between l'_i+1 and l'_i
  for(int i = 0; i < N - 1; ++i){
    // l'_i+1 - l'_i - l''_i*delta_s = 1/2 *jerk * delta_s^2
    linearMatrix(i + N*3 + N - 1, i + N + 1)   =  1.0; 
    linearMatrix(i + N*3 + N - 1, i + N)       = -1.0; 
    linearMatrix(i + N*3 + N - 1, i + N*2 + 1) =  -ds_1[i]; 
    lowerBound[i + N*3 + N - 1] = -lat_jerk_limit*ds_2[i]/2.0;
    upperBound[i + N*3 + N - 1] =  lat_jerk_limit*ds_2[i]/2.0;
  }

  //adding the equation constraint between l''_i+1 and l''_i
  for(int i = 0; i < N - 1; ++i){
    // l''_i+1 - l''_i = jerk * delta_s
    linearMatrix(i + N*3 + (N - 1)*2, i + 2*N)     = -1.0; 
    linearMatrix(i + N*3 + (N - 1)*2, i + 2*N + 1) =  1.0; 
    lowerBound[i + N*3 + (N - 1)*2] = -lat_jerk_limit*ds_1[i];
    upperBound[i + N*3 + (N - 1)*2] =  lat_jerk_limit*ds_1[i];
  }
  // std::cout << "lowerbound is :" << std::endl << lowerBound << std::endl;
  // std::cout << "upperbound is :" << std::endl << upperBound << std::endl;

  //adding the constraint of the starting point
  linearMatrix(0 + N*3 + (N - 1)*3, 0)           = 1.0; 
  // lowerBound[0 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].l;
  // upperBound[0 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].l;
  lowerBound[0 + N*3 + (N - 1)*3] = starting_l;
  upperBound[0 + N*3 + (N - 1)*3] = starting_l;

  linearMatrix(1 + N*3 + (N - 1)*3, 0 + N)       = 1.0; 
  lowerBound[1 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].dl;
  upperBound[1 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].dl;

  linearMatrix(2 + N*3 + (N - 1)*3, 0 + 2*N)     = 1.0; 
  lowerBound[2 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].ddl;
  upperBound[2 + N*3 + (N - 1)*3] = smoothed_trajectory.data_[0].ddl;

  //the curvature constraint frome Zhang Yajia
  for(int i = 0; i < N; ++i){
    //k_max*k_r*l <= k_max - |k_r|
    linearMatrix(i + N*6, i)=  kappa_limit*smoothed_trajectory.data_[i].kappa; 
    lowerBound[i + N*6] = -1e10;
    upperBound[i + N*6] = kappa_limit - std::abs(smoothed_trajectory.data_[i].kappa);
  }

  //set the matrix P(hessian matrix)
  for(int i = 0; i < N; ++i){
    hessian_.insert(i, i) = w_l + w_obs;
    gradientQ(i) += -2.0*w_obs*lMidRef[i];
  }
  for(int i = 0; i < N; ++i){
    hessian_.insert(i + N, i + N) = w_dl;
  }
  for(int i = 0; i < (N - 1); ++i){
    if(i ==0){
      hessian_.insert(i + N*2    , i + N*2) = w_ddl + w_dddl/ds_2[i];
    }
    else{
      hessian_.insert(i + N*2    , i + N*2) = w_ddl + 2.0*w_dddl/ds_2[i];
    }
    hessian_.insert(i + N*2 + 1, i + N*2) = -2.0*w_dddl/ds_2[i];
  }
  hessian_.insert(N*3 -1, N*3 -1) = w_ddl + w_dddl/ds_2[N-1];

  linearMatrix_ = linearMatrix.sparseView();
  // std::cout << "hessian" << hessian_ << std::endl;
  
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  
  solver.data()->setNumberOfVariables(decision_variable_num); 
  solver.data()->setNumberOfConstraints(constraint_num); 
  if(!solver.data()->setHessianMatrix(hessian_)) return 1;
  if(!solver.data()->setGradient(gradientQ)) return 1; 
  if(!solver.data()->setLinearConstraintsMatrix(linearMatrix_)) return 1;
  if(!solver.data()->setLowerBound(lowerBound)) return 1;
  if(!solver.data()->setUpperBound(upperBound)) return 1;

  // instantiate the solver
  if(!solver.initSolver()) return 1;

  Eigen::VectorXd originQPSolution;

  TicToc origin_qp_calculation_timer;
  origin_qp_calculation_timer.tic();
  // solve the QP problem
  if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << "Something wrong during the origin OSQP problem solving!" << std::endl;
    return 1;
  }

  // get the controller input
  originQPSolution = solver.getSolution();
  origin_qp_calculation_duration = origin_qp_calculation_timer.toc();
  std::cout << "origin_qp calculation duration is " << origin_qp_calculation_duration << " ms" << std::endl;
  UpdatePath(originQPSolution, smoothed_trajectory, origin_qp_path);
  PubSLPath(origin_qp_path,"originQP");
  std::vector<double> originQP_x, originQP_y;
  for(int i = 0; i < N; ++i){
    originQP_x.push_back(origin_qp_path.data_[i].x);
    originQP_y.push_back(origin_qp_path.data_[i].y);
  }
  visualization::Plot(originQP_x, originQP_y, 0.08, visualization::Color::Blue, 3, "Original QP Trajectory");
  visualization::Trigger();
  EvaluatePathKappa("originQP", smoothed_trajectory, originQP_x, originQP_y);

  //stage II: proposed method

  // settings
  solverNew.settings()->setVerbosity(false);
  // solverNew.settings()->setWarmStart(true);
  // // solverNew.setWarmStart(solver.data()->m_data->m);
  // solverNew.settings()->setAbsoluteTolerance(1e-6);
  // solverNew.settings()->setMaxIteration(30000);
  // solverNew.settings()->setRelativeTolerance(1e-6);
  if(QPIteration_num == 0){
    PJPOWithCurvature(smoothed_trajectory, QPIteration_num);
    VisualizeResult(N, newQPSolution);
    optimized_path_vec.resize(QPIteration_num+1);
    UpdatePath(newQPSolution, smoothed_trajectory, optimized_path_vec[QPIteration_num]);
    PubSLPath(optimized_path_vec[QPIteration_num],"newQP");
    std::vector<double> newQP_x, newQP_y;
    // std::cout << "The size of optimized_path_vec[0] is " << (int)optimized_path_vec[QPIteration_num].data_.size() << std::endl;
    for(int i = 0; i < N; ++i){
      newQP_x.push_back(optimized_path_vec[QPIteration_num].data_[i].x);
      newQP_y.push_back(optimized_path_vec[QPIteration_num].data_[i].y);
    }
    visualization::Plot(newQP_x, newQP_y, 0.06, visualization::Color::Red, 3, "New QP Trajectory");
    visualization::Trigger();
    EvaluatePathKappa("newQP", smoothed_trajectory, newQP_x, newQP_y);
    ++QPIteration_num;
  }
  {
    PJPOWithCurvature(optimized_path_vec[QPIteration_num-1], QPIteration_num);
    optimized_path_vec.resize(QPIteration_num+1);
    UpdatePath(newQPSolution, smoothed_trajectory, optimized_path_vec[QPIteration_num]);
    PubSLPath(optimized_path_vec[QPIteration_num],"newQP_iteration"  + std::to_string(QPIteration_num));
    std::vector<double> newQP_x, newQP_y;
    // std::cout << "The size of optimized_path_vec[0] is " << (int)optimized_path_vec[QPIteration_num].data_.size() << std::endl;
    for(int i = 0; i < N; ++i){
      newQP_x.push_back(optimized_path_vec[QPIteration_num].data_[i].x);
      newQP_y.push_back(optimized_path_vec[QPIteration_num].data_[i].y);
    }
    visualization::Plot(newQP_x, newQP_y, 0.06, visualization::Color::Magenta, 4, "New QP Trajectory Iteration "  + std::to_string(QPIteration_num));
    visualization::Trigger();
    EvaluatePathKappa("newQP_iteration"  + std::to_string(QPIteration_num), smoothed_trajectory, newQP_x, newQP_y);
    ++QPIteration_num;
  }
  {
    PJPOWithCurvature(optimized_path_vec[QPIteration_num-1], QPIteration_num);
    optimized_path_vec.resize(QPIteration_num+1);
    UpdatePath(newQPSolution, smoothed_trajectory, optimized_path_vec[QPIteration_num]);
    PubSLPath(optimized_path_vec[QPIteration_num],"newQP_iteration"  + std::to_string(QPIteration_num));
    std::vector<double> newQP_x, newQP_y;
    // std::cout << "The size of optimized_path_vec[0] is " << (int)optimized_path_vec[QPIteration_num].data_.size() << std::endl;
    for(int i = 0; i < N; ++i){
      newQP_x.push_back(optimized_path_vec[QPIteration_num].data_[i].x);
      newQP_y.push_back(optimized_path_vec[QPIteration_num].data_[i].y);
    }
    visualization::Plot(newQP_x, newQP_y, 0.06, visualization::Color::Green, 5, "New QP Trajectory Iteration "  + std::to_string(QPIteration_num));
    visualization::Trigger();
    EvaluatePathKappa("newQP_iteration"  + std::to_string(QPIteration_num), smoothed_trajectory, newQP_x, newQP_y);
    ++QPIteration_num;
  }
  {
    PJPOWithCurvature(optimized_path_vec[QPIteration_num-1], QPIteration_num);
    optimized_path_vec.resize(QPIteration_num+1);
    UpdatePath(newQPSolution, smoothed_trajectory, optimized_path_vec[QPIteration_num]);
    PubSLPath(optimized_path_vec[QPIteration_num],"newQP_iteration"  + std::to_string(QPIteration_num));
    std::vector<double> newQP_x, newQP_y;
    // std::cout << "The size of optimized_path_vec[0] is " << (int)optimized_path_vec[QPIteration_num].data_.size() << std::endl;
    for(int i = 0; i < N; ++i){
      newQP_x.push_back(optimized_path_vec[QPIteration_num].data_[i].x);
      newQP_y.push_back(optimized_path_vec[QPIteration_num].data_[i].y);
    }
    visualization::Plot(newQP_x, newQP_y, 0.06, visualization::Color::Green, 7, "New QP Trajectory Iteration "  + std::to_string(QPIteration_num));
    visualization::Trigger();
    EvaluatePathKappa("newQP_iteration"  + std::to_string(QPIteration_num), smoothed_trajectory, newQP_x, newQP_y);
    ++QPIteration_num;
  }
  {
    PJPOWithCurvature(optimized_path_vec[QPIteration_num-1], QPIteration_num);
    optimized_path_vec.resize(QPIteration_num+1);
    UpdatePath(newQPSolution, smoothed_trajectory, optimized_path_vec[QPIteration_num]);
    PubSLPath(optimized_path_vec[QPIteration_num],"newQP_iteration"  + std::to_string(QPIteration_num));
    std::vector<double> newQP_x, newQP_y;
    // std::cout << "The size of optimized_path_vec[0] is " << (int)optimized_path_vec[QPIteration_num].data_.size() << std::endl;
    for(int i = 0; i < N; ++i){
      newQP_x.push_back(optimized_path_vec[QPIteration_num].data_[i].x);
      newQP_y.push_back(optimized_path_vec[QPIteration_num].data_[i].y);
    }
    visualization::Plot(newQP_x, newQP_y, 0.06, visualization::Color::Green, 7, "New QP Trajectory Iteration "  + std::to_string(QPIteration_num));
    visualization::Trigger();
    EvaluatePathKappa("newQP_iteration"  + std::to_string(QPIteration_num), smoothed_trajectory, newQP_x, newQP_y);
    ++QPIteration_num;
  }
  
  return true;

}

bool PJPOCurvature::EvaluatePathKappa(std::string path_name, DiscretizedTrajectory & reference_path, 
                          const std::vector<double> & vec_x, const std::vector<double> & vec_y){
  std::vector<double> vec_s, vec_kappa, vec_kappa_smoothed, vec_dkappa;
  int N = reference_path.data_.size();
  vec_s.push_back(0);
  vec_kappa.push_back(0); vec_kappa_smoothed.push_back(0);
  vec_dkappa.push_back(0);
  for(int i = 1; i < N-1; ++i){
    //reference https://www.matongxue.com/madocs/2106/
    Eigen::Vector2d p1, p2, p3, v1, v2, v3;
    p1 << vec_x[i-1], vec_y[i-1];
    p2 << vec_x[i], vec_y[i];
    p3 << vec_x[i+1], vec_y[i+1];
    double current_kappa = CalculateKappa(p1, p2, p3);
    vec_s.push_back(reference_path.data_[i].s);
    vec_kappa.push_back(current_kappa);
  }
  vec_s.push_back(reference_path.data_[N-1].s);
  vec_kappa.push_back(0);
  for(int i = 1; i < N-1; ++i){
    vec_kappa_smoothed.push_back(vec_kappa[i]);
    vec_dkappa.push_back( (vec_kappa_smoothed[i] - vec_kappa_smoothed[i-1]) / (vec_s[i] - vec_s[i-1]) ); 
  }
  vec_kappa_smoothed.push_back(0);
  vec_dkappa.push_back((vec_kappa[N-1] - vec_kappa[N-2]) / (vec_s[N-1] - vec_s[N-2]));
  geometry_msgs::PoseArray vec_kappa_info;
  vec_kappa_info.header.frame_id = path_name;
  vec_kappa_info.poses.reserve(N);
  for(unsigned int i = 0; i < N; ++i){
    //the pose.position.x, y, z from  vec_kappa_info represents the s, kappa, dkappa of each point, respectively
    geometry_msgs::Pose pose;
    pose.position.x = vec_s[i];
    pose.position.y = vec_kappa_smoothed[i];
    pose.position.z = vec_dkappa[i];
    vec_kappa_info.poses.push_back(pose);
  }
  DiscretizedTrajectory result_path;
  result_path.data_.resize(N);
  for(int i = 0; i < N; ++i){
    result_path.data_[i].x = vec_x[i];
    result_path.data_[i].y = vec_y[i];
  }
  visualization::PubKappaInfo(vec_kappa_info);
  return true;
}

bool PJPOCurvature::PubSLPath(const DiscretizedTrajectory & path, std::string path_id){
  pjpo_curvature::FrenetPath frenet_path;
  frenet_path.id = path_id;
  for(int i = 0; i < path.data_.size(); ++i){
    FrenetPoint pt;
    pt.x = path.data_[i].x;
    pt.y = path.data_[i].y;
    pt.s = path.data_[i].s;
    pt.l = path.data_[i].l;
    pt.dl = path.data_[i].dl;
    pt.ddl = path.data_[i].ddl;
    // pt.dddl = path.data_[i].dddl;
    pt.kappa = path.data_[i].kappa;
    pt.dkappa = path.data_[i].dkappa;
    pt.left_bound = path.data_[i].left_bound;
    pt.right_bound = path.data_[i].right_bound;
    pt.left_bound_x = smoothed_trajectory.data_[i].x - smoothed_trajectory.data_[i].left_bound*sin(smoothed_trajectory.data_[i].theta);
    pt.left_bound_y = smoothed_trajectory.data_[i].y + smoothed_trajectory.data_[i].left_bound*cos(smoothed_trajectory.data_[i].theta);
    pt.right_bound_x = smoothed_trajectory.data_[i].x - smoothed_trajectory.data_[i].right_bound*sin(smoothed_trajectory.data_[i].theta);
    pt.right_bound_y = smoothed_trajectory.data_[i].y + smoothed_trajectory.data_[i].right_bound*cos(smoothed_trajectory.data_[i].theta);
    frenet_path.path.push_back(pt);
  }
  visualization::PubSLPath(frenet_path);
  return true;
}

double PJPOCurvature::CalculateKappa(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3){
  //reference https://www.matongxue.com/madocs/2106/
  double current_kappa = 0, radius;
  Eigen::Vector2d v1, v2, v3;
  v1 = p2 - p1;
  v2 = p3 - p2;
  v3 = p3 - p1;
  double cosine_value = v1.dot(v2);
  if( (v1.norm() * v2.norm() - std::abs(cosine_value))  < 1e-8){
    //three point in one line
    return 0.0;
  }
  else{
    double square = v1[0] * v2[1] - v2[0] * v1[1];
    radius = v1.norm() * v2.norm() * v3.norm() / (2.0 * square);
    current_kappa = 1.0/radius;
    // std::cout << "current_kappa: " << current_kappa << std::endl;
  }
  return current_kappa;
}

bool PJPOCurvature::PJPOWithCurvature(const DiscretizedTrajectory & initial_path,  int iteration){
  int N = initial_path.data_.size();
  //set P matrix（hessian matrix)
  int decision_variable_num = N * 3; // (l, dl, ddl) * N
  int constraint_num = 0;
  constraint_num += N*3; 
  constraint_num += N - 1; 
  constraint_num += N - 1; 
  constraint_num += N - 1; 
  constraint_num += 3;
  constraint_num += N ;
  constraint_num += N ;
  // constraint_num += N-1 ;
  hessianNew.resize(decision_variable_num, decision_variable_num);
  hessianNew.setZero();
  gradientQNew.resize(decision_variable_num);
  gradientQNew.setZero();
  
  for(int i = 0; i < N; ++i){
    hessianNew(i, i) = w_l + w_obs;
    gradientQNew(i) += -2.0*w_obs*lMidRef[i];
  }
  for(int i = 0; i < N; ++i){
    hessianNew(i + N, i + N) = w_dl;
  }
  hessianNew(0 + N*2    , 0 + N*2) = w_ddl + w_dddl/ds_2[0];
  for(int i = 1; i < (N - 1); ++i){
    hessianNew(i + N*2    , i + N*2) = w_ddl + 2.0*w_dddl/ds_2[i];
    hessianNew(i + N*2 + 1, i + N*2) = -2.0*w_dddl/ds_2[i];
  }
  hessianNew(N*3 -1, N*3 -1) = w_ddl + w_dddl/ds_2[N - 1];
  //adding curvature-related kernals in objective function
  static TicToc kappa_calculation_timer;
  kappa_calculation_timer.tic();
  
  std::vector<double> const_1;
  std::vector<double> const_2;
  const_1.resize(N);
  const_2.resize(N);
  Avec_.resize(N);
  Cvec_.resize(N);
  A_vec_.resize(N);
  C_vec_.resize(N);
  A_cross_vec_.resize(N - 1);
  C_cross_vec_.resize(N - 1);
  for(int i = 0; i < N; ++i){
    TrajectoryPoint cur_pt = initial_path.data_[i];
    // cur_pt.l += 0.5;
    double theta = atan2(cur_pt.dl, 1 - cur_pt.kappa*cur_pt.l);
    double kappa_ref = cur_pt.kappa;
    double kr_1 = kappa_ref;
    double kr_2 = pow(kr_1, 2);
    double kr_3 = pow(kr_1, 3);
    double dkr = cur_pt.dkappa;
    
    double krl_1 = 1- kappa_ref*cur_pt.l;
    double krl_2 = pow(krl_1, 2);
    double krl_3 = pow(krl_1, 3);
    double krl_4 = pow(krl_1, 4);
    double c_1 = cos(theta);
    // std::cout << "theta " << theta << ", c_1 " << c_1<< std::endl;
    double c_2 = pow(c_1, 2);
    double c_3 = pow(c_1, 3);
    double l = cur_pt.l;
    double dl = cur_pt.dl;
    double ddl = cur_pt.ddl;
    double kappa_real = ((ddl + 
                        (dkr*l + kr_1*dl)*tan(theta)*c_2/krl_1
                        + kappa_ref)*c_1/krl_1 );
    double kappa_real_2 = pow(kappa_real, 2);
    double kappa_estimate_constant = 0;
    // calculate the first ofder coefficient of l_i, dl_i, ddl_i
    Cvec_[i](0) = c_1*kr_2/krl_2 + (-2.0*c_1*kr_3*l)/krl_3;
    Cvec_[i](1) = c_3*l*dkr/krl_3 + (-dkr*l*(c_3 + 2*c_3*l*kr_1))/krl_4;
    Cvec_[i](2) = 2.0*c_3/krl_2 + (-2.0*c_3*kr_1*l)/krl_3 + 3.0*c_2*kr_2*std::pow(l, 2)/krl_4;
    // calculate the second ofder coefficient of l_i, dl_i, ddl_i
    Avec_[i] << 0,0,0,
       0,0,0,
       0,0,0;
    Avec_[i](0, 0) = c_1*kr_3/krl_3;
    // A(0, 0) = A(0, 0);
    Avec_[i](1, 1) = c_3*kr_1/krl_3 + c_3*kr_1/krl_3;
    Avec_[i](1, 0) = (dkr*(c_3 + 2.0*c_3*l*kr_1)/krl_4 + c_3*dkr/krl_3);
    Avec_[i](2, 0) = (2.0*c_3*kr_1/krl_3);

    // initial state L_0 = [ l, dl, ddl]
    Eigen::Vector3d L_0;
    L_0[0] = l;
    L_0[1] = dl;
    L_0[2] = ddl;
    // kappa_estimate = kappa_estimate_constant + 1/2*L^TAL + C^TL
    double kappa_estimate_zero = kappa_estimate_constant + 1.0/2.0*L_0.transpose() *Avec_[i]* L_0 + Cvec_[i].transpose() * L_0;
    double kappa_estimate_zero_diff = kappa_real - kappa_estimate_zero;

    const_1[i] = kappa_estimate_constant;
    const_1[i] += kappa_estimate_zero_diff;
    // std::cout << const_1[i] << std::endl;
    const_2[i] = pow(const_1[i], 2);
    // kappa_estimate_2 = const^2 + L^T(CC^T+2const*A)L + 2constC^TL
    // A_ = CC^T+2const*A
    // C_ = 2*const*C
    A_vec_[i] = Cvec_[i] * Cvec_[i].transpose() + const_1[i]*Avec_[i];
    C_vec_[i] = 2.0 * const_1[i] * Cvec_[i];
  }
  //adding kappa kernal
  for(int i = 0; i < N - 1; ++i){
    // A_cross_vec_[i] = Eigen::Matrix<double, 6, 6>::Zero();
    // C_cross_vec_[i] = Eigen::Matrix<double, 6, 1>::Zero();
    A_cross_vec_[i].block<3, 3>(0, 0) = -2.0*1.0/2.0*const_1[i + 1]*Avec_[i];
    A_cross_vec_[i].block<3, 3>(3, 3) = -2.0*1.0/2.0*const_1[i]*Avec_[i + 1];
    A_cross_vec_[i].block<3, 3>(3, 0) = -2.0*Cvec_[i] * Cvec_[i+1].transpose();
    A_cross_vec_[i].block<3, 3>(0, 0) += A_vec_[i];
    A_cross_vec_[i].block<3, 3>(3, 3) += A_vec_[i + 1];
    C_cross_vec_[i].head(3) = -2.0*const_1[i + 1] * Cvec_[i];
    C_cross_vec_[i].tail(3) = -2.0*const_1[i] * Cvec_[i + 1];
    C_cross_vec_[i].head(3) += C_vec_[i];
    C_cross_vec_[i].tail(3) += C_vec_[i + 1];
    for(int j = 0; j < 3; ++j){
      for(int k = 0; k < 3; ++k){
        hessianNew(i + j*N, i + k*N) += w_kappa_2 * A_vec_[i](j, k);
        // if(abs(A_(j, k)) > 1e-5 ) std::cout <<"!!! A_(j, k) Value is " <<A_(j, k) <<std::endl;
      }
      gradientQNew(i + j*N) += w_kappa_2 * C_vec_[i](j);
    }

    // adding the cost of the（dkappa）^2 term
    // for(int m = 0; m < 2; ++m){
    //   for(int j = 0; j < 3; ++j){
    //     for(int k = 0; k < 3; ++k){
    //       hessianNew(i + m + j*N, i + m + k*N) += w_dkappa/ds_2[i] * C_cross_vec_[i](j + m*3, k + m*3);
    //     }
    //     gradientQNew(i + m + j*N) += w_dkappa/ds_2[i] * C_cross_vec_[i](j + m*3);
    //   }
    // }
  }

  //the previous curvature linear constraint method from Zhang Yajia
  for(int i = 0; i < N; ++i){
    //k_max*k_r*l <= k_max - |k_r|
    linearMatrix(i + N*6, i)=  kappa_limit*initial_path.data_[i].kappa; 
    lowerBound[i + N*6] = -1e10;
    upperBound[i + N*6] = kappa_limit - std::abs(initial_path.data_[i].kappa);
  }
  // the new curvature linear constraint method
  // for(int i = 0; i < N; ++i){
  //   linearMatrix(i + N*7, i)       =  1.0*C_vec_[i](0); 
  //   linearMatrix(i + N*7, i + N)   =  1.0*C_vec_[i](1); 
  //   linearMatrix(i + N*7, i + N*2) =  1.0*C_vec_[i](2); 
  //   lowerBound[i + N*7] = -kappa_limit - const_1[i];
  //   upperBound[i + N*7] =  kappa_limit - const_1[i];
  // }

  // adding the dkappa inequation constraints
  // for(int i = 0; i < N-1; ++i){
  //   // -dkappa_limit*delta_s <= kappa_i+1 - kappa_i <= dkappa_limit*delta_s
  //   linearMatrix(i + N*8, i)       =  -1.0*C_vec_[i](0); 
  //   linearMatrix(i + N*8, i + N)   =  -1.0*C_vec_[i](1); 
  //   linearMatrix(i + N*8, i + N*2) =  -1.0*C_vec_[i](2); 
  //   linearMatrix(i + N*8, i + 1)       =   1.0*C_vec_[i+1](0); 
  //   linearMatrix(i + N*8, i + N + 1)   =   1.0*C_vec_[i+1](1); 
  //   linearMatrix(i + N*8, i + N*2 + 1) =   1.0*C_vec_[i+1](2); 
  //   lowerBound[i + N*8] = (-dkappa_limit - const_1[i+1] + const_1[i])*ds_1[i];
  //   upperBound[i + N*8] = ( dkappa_limit - const_1[i+1] + const_1[i])*ds_1[i];
  // }
  auto kappa_calculation_duration = kappa_calculation_timer.toc();

  std::cout << "extra calculation duration for kappa^2 is " << kappa_calculation_duration << " ms" << std::endl;

  hessianNewSparse = hessianNew.sparseView();
  linearMatrix_ = linearMatrix.sparseView();
    
  // std::cout <<"hessianNewSparse is :"<< hessianNewSparse <<std::endl;
  // std::cout <<"gradientQNew is :"<< gradientQNew <<std::endl;
  // std::cout << "hessianNewSparse" << hessianNewSparse << std::endl;
  if(iteration == 0){
    // set the initial data of the QP solver
    solverNew.settings()->setWarmStart(true);
    solverNew.data()->setNumberOfVariables(decision_variable_num); 
    solverNew.data()->setNumberOfConstraints(constraint_num); 
    solverNew.data()->clearHessianMatrix();
    solverNew.data()->clearLinearConstraintsMatrix();
    if(!solverNew.data()->setHessianMatrix(hessianNewSparse)) return 1;
    if(!solverNew.data()->setGradient(gradientQNew)) return 1; 
    if(!solverNew.data()->setLinearConstraintsMatrix(linearMatrix_)) return 1;
    if(!solverNew.data()->setLowerBound(lowerBound)) return 1;
    if(!solverNew.data()->setUpperBound(upperBound)) return 1;
    
    // instantiate the solver
    if(solverNew.isInitialized()){
      solverNew.clearSolver();
    }
    if(!solverNew.initSolver()) return 1;
  }
  else{
    if(!solverNew.updateHessianMatrix(hessianNewSparse)) return 1;
    if(!solverNew.updateGradient(gradientQNew)) return 1;
    if(!solverNew.updateLinearConstraintsMatrix(linearMatrix_)) return 1;
    if(!solverNew.updateBounds(lowerBound, upperBound)) return 1;
  }
  // if(!solver.updateHessianMatrix(hessianNewSparse)) return 1;
  // if(!solver.updateGradient(gradientQNew)) return 1;
  // if(!solver.updateLinearConstraintsMatrix(linearMatrix_)) return 1;
  // if(!solver.updateBounds(lowerBound, upperBound)) return 1;

  TicToc qp_calculation_timer;
  qp_calculation_timer.tic();
  // if(iteration != 0){
  //   solverNew.setPrimalVariable(newQPSolution);
  // }
  
  // solve the QP problem
  if(solverNew.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
    std::cout << "Something wrong during the new OSQP problem solving!" << std::endl;
    return 1;
  }
  // get the controller input
  newQPSolution = solverNew.getSolution();
  new_qp_calculation_duration = qp_calculation_timer.toc();
  std::cout << "The "<< iteration <<" iteration qp calculation duration is " << new_qp_calculation_duration << " ms" << std::endl;
  return true;
}

void PJPOCurvature::VisualizeResult(int N, const Eigen::VectorXd & newQPSolution){
  geometry_msgs::PoseArray l_result, dl_result, ddl_result, dddl_result;
  l_result.header.frame_id = "map";
  dl_result.header.frame_id = "map";
  ddl_result.header.frame_id = "map";
  dddl_result.header.frame_id = "map";
  l_result.poses.reserve(N);
  dl_result.poses.reserve(N);
  ddl_result.poses.reserve(N);
  ddl_result.poses.reserve(N);

  for(int i = 0; i < N; ++i){
    geometry_msgs::Pose pose;
    pose.position.x = i*ds_1[0]/10.0;
    pose.position.y = newQPSolution[i]*10.0;
    l_result.poses.push_back(pose);
    pose.position.y = newQPSolution[i + N]*10.0;
    dl_result.poses.push_back(pose);
    pose.position.y = newQPSolution[i + N*2]*10.0;
    ddl_result.poses.push_back(pose);
    if(i < N -1 ){
      pose.position.y = (newQPSolution[i + 1 + N*2] - newQPSolution[i + N*2])/ds_1[i]*100.0;
      dddl_result.poses.push_back(pose);
    }
  }
  visualization::SLdraw(l_result, dl_result, ddl_result, dddl_result);
}

bool PJPOCurvature::UpdatePath(Eigen::VectorXd & newQPSolution, const DiscretizedTrajectory & reference_path, DiscretizedTrajectory & optimized_path){
  int N = newQPSolution.size()/3;
  optimized_path.data_.resize(N);
  for(int i = 0; i < N; ++i){
    optimized_path.data_[i] = reference_path.data_[i];
    optimized_path.data_[i].s = reference_path.data_[i].s;
    optimized_path.data_[i].ds = reference_path.data_[i].ds;
    optimized_path.data_[i].x = reference_path.data_[i].x - newQPSolution[i]*sin(reference_path.data_[i].theta);
    optimized_path.data_[i].y = reference_path.data_[i].y + newQPSolution[i]*cos(reference_path.data_[i].theta);
    optimized_path.data_[i].l = newQPSolution[i];
    optimized_path.data_[i].dl = newQPSolution[i+N];
    optimized_path.data_[i].ddl = newQPSolution[i+N*2];
  }
  for(int i = 1; i < N-1; ++i){
    //reference https://www.matongxue.com/madocs/2106/
    Eigen::Vector2d p1, p2, p3, v1, v2, v3;
    p1 << optimized_path.data_[i-1].x, optimized_path.data_[i-1].y;
    p2 << optimized_path.data_[i].x,   optimized_path.data_[i].y;
    p3 << optimized_path.data_[i+1].x, optimized_path.data_[i+1].y;
    optimized_path.data_[i].kappa = CalculateKappa(p1, p2, p3);
  }
  for(int i = 1; i < N; ++i){
    optimized_path.data_[i].dkappa = (optimized_path.data_[i].kappa - optimized_path.data_[i-1].kappa)
                                    /(optimized_path.data_[i].s - optimized_path.data_[i-1].s);
  }
  return true;
}

} //namespace