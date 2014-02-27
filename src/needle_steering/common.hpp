#pragma once

#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/modeling.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/bullet_collision_checker.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/common.hpp"
#include "trajopt/plot_callback.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "utils/clock.hpp"
#include "utils/config.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <ctime>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include "strtk.hpp"
#include <math.h>

#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
  #include <direct.h>
  #define GetCurrentDir _getcwd
#else
  #include <unistd.h>
  #define GetCurrentDir getcwd
#endif


//#define CHANNEL

//#define NEEDLE_TEST
//#define USE_CURVATURE

#define SWITCH_DEFAULT default: \
                         PRINT_AND_THROW("not implemented");

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace Eigen;

namespace Needle {
  typedef Matrix<double, 1, 1> Vector1d;

  typedef Matrix<double, 6, 1> Vector6d;

  //typedef NeedleSQP OptimizerT;
  typedef BasicTrustRegionSQP OptimizerT;
  //typedef LineSearchSQP OptimizerT;

  inline double bound_inf(double result, double bound);

  inline string get_current_directory() {
    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) {
      throw std::runtime_error("cannot get current path");
    }
    return string(cCurrentPath);
  }

  
}
