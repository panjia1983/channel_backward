#pragma once

#include "bsp/bsp.hpp"
#include "bsp/openrave_utils.hpp"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/problem_description.hpp"
#include "geometry_2d.hpp"
#include <QtCore>
#include <QImage>
#include <qevent.h>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include <array>

using namespace BSP;
using namespace Geometry2D;

namespace BarrettRobotBSP {

  // This will generate a bunch of types like StateT, ControlT, etc.
  BSP_TYPEDEFS(
      7, // state_dim
      7, // state_noise_dim
      7, // control_dim
      3, // observe_dim
      3, // observe_noise_dim
      28, // sigma_dof
      35 // belief_dim
  );

  typedef Matrix<double, 7, 1> Vector7d;
  typedef Matrix<double, 7, 7> Matrix7d;

  // state: { {robot_dofs} }
  // control: { {d_robot_dofs} }
  // observation: { (whatever) }

  class BarrettRobotBSPProblemHelper;
  typedef boost::shared_ptr<BarrettRobotBSPProblemHelper> BarrettRobotBSPProblemHelperPtr;

  class BarrettRobotStateFunc : public StateFunc<StateT, ControlT, StateNoiseT> {
  public:
    typedef boost::shared_ptr<BarrettRobotStateFunc> Ptr;
    BarrettRobotBSPProblemHelperPtr barrett_robot_helper;

    BarrettRobotStateFunc();
    BarrettRobotStateFunc(BSPProblemHelperBasePtr helper);
    StateT operator()(const StateT& x, const ControlT& u, const StateNoiseT& m) const ;
  };

  class BarrettRobotObserveFunc : public ObserveFunc<StateT, ObserveT, ObserveNoiseT> {
  public:
    typedef boost::shared_ptr<BarrettRobotObserveFunc> Ptr;
    BarrettRobotBSPProblemHelperPtr barrett_robot_helper;
    BarrettRobotObserveFunc();
    BarrettRobotObserveFunc(BSPProblemHelperBasePtr helper);
    ObserveT operator()(const StateT& x, const ObserveNoiseT& n) const;
  };

  class BarrettRobotBeliefFunc : public EkfBeliefFunc<BarrettRobotStateFunc, BarrettRobotObserveFunc, BeliefT> {
  public:
    typedef boost::shared_ptr<BarrettRobotBeliefFunc> Ptr;
    BarrettRobotBSPProblemHelperPtr barrett_robot_helper;
    BarrettRobotBeliefFunc();
    BarrettRobotBeliefFunc(BSPProblemHelperBasePtr helper, StateFuncPtr f, ObserveFuncPtr h);
  };

  class BarrettRobotBSPProblemHelper : public BSPProblemHelper<BarrettRobotBeliefFunc> {
  public:
    typedef typename BeliefConstraint<BarrettRobotBeliefFunc>::Ptr BeliefConstraintPtr;
    void add_goal_constraint(OptProb& prob);
    void add_collision_term(OptProb& prob);
    void configure_problem(OptProb& prob);
    BarrettRobotBSPProblemHelper();

    RobotBasePtr robot;
    RobotAndDOFPtr rad;
    KinBody::LinkPtr link;
    Matrix4d goal_trans;
  };

  class BarrettRobotBSPPlanner : public BSPPlanner<BarrettRobotBSPProblemHelper> {
  public:
    void initialize();
    BarrettRobotBSPPlanner();
    void initialize_optimizer_parameters(BSPTrustRegionSQP& opt, bool is_first_time=true);
    RobotBasePtr robot;
    RobotAndDOFPtr rad;
    Matrix4d goal_trans;
    KinBody::LinkPtr link;
  };

  typedef boost::shared_ptr<BarrettRobotBSPPlanner> BarrettRobotBSPPlannerPtr;


}
