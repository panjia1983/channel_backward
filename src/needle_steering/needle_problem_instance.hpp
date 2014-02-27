#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "local_configuration.hpp"

namespace Needle {
  struct NeedleProblemInstance {
    double Delta_lb;
    Vector6d start;
    Vector6d goal;
    VarArray twistvars;
    VarArray phivars;
    VarArray curvature_or_radius_vars;
    VarArray Deltavars;
    Var Deltavar;
    vector<LocalConfigurationPtr> local_configs;
    vector<ConstraintPtr> dynamics_constraints;
    vector<ConstraintPtr> collision_constraints;
    NeedleProblemHelperPtr helper;
    DblVec initVec;
    int T;
    int id;
    Vector3d start_position_error_relax;
    double start_orientation_error_relax;
    double goal_distance_error_relax;

    VectorXd GetSolution(OptimizerT& opt);
    vector<Vector6d> GetStates(OptimizerT& opt);
    void SetSolution(const VectorXd& sol, OptimizerT& opt);
    VectorXd GetSolutionWithoutFirstTimestep(const VectorXd& sol);
    void PrintSolutionTrajectory(const VectorXd& sol);
  };
}
