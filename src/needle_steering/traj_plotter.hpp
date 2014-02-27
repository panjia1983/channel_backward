#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_planner.hpp"

namespace Needle {
  struct TrajPlotter {
    TrajPlotter();
    bool OptimizerCallback(OptProb*, DblVec& x, NeedleProblemHelperPtr helper, NeedleProblemPlannerPtr planner, bool halt, const vector< vector<Vector6d> >& extra_states);
  };

  struct NeedleSimPlotter {
    NeedleSimPlotter();
    void Plot(NeedleProblemPlannerPtr planner);
  };
}
