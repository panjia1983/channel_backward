#pragma once

#include "sco/expr_ops.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/modeling.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/bullet_collision_checker.hpp"
#include "trajopt/collision_checker.hpp"
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
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/static_assert.hpp>
#include <boost/any.hpp>
#include <boost/type_traits.hpp>
#include <ctime>
#include <vector>
#include <deque>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include <assert.h>

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace sco;
using namespace Eigen;

#define DEBUG

// constants
namespace BSP {
  const double DefaultEpsilon = 0.0009765625;
  const double eps = 1e-5;
}
