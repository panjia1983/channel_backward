#include "needle_problem_instance.hpp"
#include "utils.hpp"

namespace Needle {

  VectorXd NeedleProblemInstance::GetSolution(OptimizerT& opt) {
    DblVec& x = opt.x(); 
    // first stack poses
    VectorXd ret;
    for (int i = 0; i < local_configs.size(); ++i) {
      ret = concat(ret, logDown(local_configs[i]->pose));
    }
    if (twistvars.size()) {
      ret = concat(ret, getVec(x, twistvars.flatten()));
    }
    if (phivars.size()) {
      ret = concat(ret, getVec(x, phivars.flatten()));
    }
    if (Deltavars.size()) {
      ret = concat(ret, getVec(x, Deltavars.flatten()));
    }
    if (Deltavar.var_rep) {
      ret = concat(ret, getVec(x, singleton<Var>(Deltavar)));
    }
    if (curvature_or_radius_vars.size()) {
      ret = concat(ret, getVec(x, curvature_or_radius_vars.flatten()));
    }
    return ret;
  }

  vector<Vector6d> NeedleProblemInstance::GetStates(OptimizerT& opt) {
    DblVec& x = opt.x(); 
    vector<Vector6d> ret;
    for (int i = 0; i < local_configs.size(); ++i) {
      ret.push_back(logDown(local_configs[i]->pose * expUp(getVec(x, twistvars.row(i)))));
    }
    return ret;
  }

  void NeedleProblemInstance::PrintSolutionTrajectory(const VectorXd& sol) {
    int offset = 0;
    cout << "solution trajectory: " << endl;
    for (int i = 0; i < local_configs.size(); ++i) {
      cout << sol.middleRows(offset, 6).transpose() << endl;
      offset += 6;
    }
    if (twistvars.size()) {
      cout << "solution twists: " << sol.middleRows(offset, twistvars.size()).transpose() << endl;
      offset += twistvars.size();
    }
    if (phivars.size()) {
      cout << "solution phis: " << sol.middleRows(offset, phivars.size()).transpose() << endl;
      offset += phivars.size();
    }
    if (Deltavars.size()) {
      cout << "solution Deltas: " << sol.middleRows(offset, Deltavars.size()).transpose() << endl;
      offset += Deltavars.size();
    }
    if (Deltavar.var_rep) {
      cout << "solution Delta: " << sol.middleRows(offset, 1).transpose() << endl;
      offset += 1;
    }
    if (curvature_or_radius_vars.size()) {
      cout << "solution curvature or radius: " << sol.middleRows(offset, curvature_or_radius_vars.size()).transpose() << endl;
      offset += curvature_or_radius_vars.size();
    }


  }

  VectorXd NeedleProblemInstance::GetSolutionWithoutFirstTimestep(const VectorXd& sol) {
    DblVec ret;
    int offset = 0;
    offset += 6;
    for (int i = 1; i < local_configs.size(); ++i) {
      ret = concat(ret, toDblVec(sol.middleRows(offset, 6)));
      offset += 6;
    }
    if (twistvars.size()) {
      ret = concat(ret, toDblVec(sol.middleRows(offset + 6, twistvars.size() - 6)));
      offset += twistvars.size();
    }
    if (phivars.size()) {
      ret = concat(ret, toDblVec(sol.middleRows(offset + 1, phivars.size() - 1)));
      offset += phivars.size();
    }
    if (Deltavars.size()) {
      ret = concat(ret, toDblVec(sol.middleRows(offset + 1, Deltavars.size() - 1)));
      offset += Deltavars.size();
    }
    if (Deltavar.var_rep) {
      ret = concat(ret, toDblVec(sol.middleRows(offset, 1)));
      offset += 1;
    } 
    if (curvature_or_radius_vars.size()) {
      ret = concat(ret, toDblVec(sol.middleRows(offset + 1, curvature_or_radius_vars.size() - 1)));
      offset += curvature_or_radius_vars.size();
    }
    return toVectorXd(ret);
  }

  void NeedleProblemInstance::SetSolution(const VectorXd& sol, OptimizerT& opt) {
    int offset = 0;
    DblVec& x = opt.x();
    for (int i = 0; i < local_configs.size(); ++i) {
      local_configs[i]->pose = expUp(sol.middleRows(offset, 6));
      offset += 6;
    }
    if (twistvars.size()) {
      setVec(x, twistvars.flatten(), sol.middleRows(offset, twistvars.size()));
      offset += twistvars.size();
    }
    if (phivars.size()) {
      setVec(x, phivars.flatten(), sol.middleRows(offset, phivars.size()));
      offset += phivars.size();
    }
    if (Deltavars.size()) {
      setVec(x, Deltavars.flatten(), sol.middleRows(offset, Deltavars.size()));
      offset += Deltavars.size();
    }
    if (Deltavar.var_rep) {
      setVec(x, singleton<Var>(Deltavar), sol.middleRows(offset, 1));
      offset += 1;
    }
    if (curvature_or_radius_vars.size()) {
      setVec(x, curvature_or_radius_vars.flatten(), sol.middleRows(offset, curvature_or_radius_vars.size()));
      offset += curvature_or_radius_vars.size();
    }
  }
}
