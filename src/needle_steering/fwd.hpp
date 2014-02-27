#pragma once

#include "common.hpp"

namespace Needle {
  struct NeedleProblemHelper;
  typedef boost::shared_ptr<NeedleProblemHelper> NeedleProblemHelperPtr;

  struct NeedleProblemInstance;
  typedef boost::shared_ptr<NeedleProblemInstance> NeedleProblemInstancePtr;

  struct NeedleProblemPlanner;
  typedef boost::shared_ptr<NeedleProblemPlanner> NeedleProblemPlannerPtr;

  struct TrajPlotter;
  struct NeedleSimPlotter;

  struct LocalConfiguration;
  typedef boost::shared_ptr<LocalConfiguration> LocalConfigurationPtr;
}
