#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "local_configuration.hpp"

namespace Needle {
  struct NeedleProblemInstance {
    double Delta_lb;
    Vector6d entry;
    Vector6d final;
    vector<Vector6d> init_traj;
    vector<VectorXd> init_control;
    VarArray twistvars;
    VarArray phivars;
    VarArray curvature_vars;
    Var Deltavar;
    vector<LocalConfigurationPtr> local_configs;
    vector<ConstraintPtr> dynamics_constraints;
    vector<ConstraintPtr> collision_constraints;
    NeedleProblemHelperPtr helper;
    DblVec initVec;
    int T;
    int id;
    Vector3d entry_position_error_relax;
    double entry_orientation_error_relax;
    double final_distance_error_relax;

    VectorXd GetSolution(OptimizerT& opt);
    vector<Vector6d> GetStates(OptimizerT& opt);
    void SetSolution(const VectorXd& sol, OptimizerT& opt);
    VectorXd GetSolutionWithoutFirstTimestep(const VectorXd& sol);
    void PrintSolutionTrajectory(const VectorXd& sol);
  };
}
