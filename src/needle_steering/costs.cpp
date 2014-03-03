#include "costs.hpp"
#include "utils.hpp"
#include "needle_problem_instance.hpp"
#include "needle_problem_helper.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/collision_checker.hpp"

namespace Needle {

  ConstantSpeedCost::ConstantSpeedCost(const Var& var, double coeff, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi) : Cost("Speed"), var(var), coeff(coeff), helper(helper), pi(pi) {
    exprInc(expr, exprMult(var, coeff * pi->T));
  }

  double ConstantSpeedCost::value(const vector<double>& xvec, Model* model) {
    double speed = getVec(xvec, singleton<Var>(var))[0];
    return speed * coeff * pi->T;
  }

  ConvexObjectivePtr ConstantSpeedCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addAffExpr(expr);
    return out;
  }

  DiffGeometryQuadraticCost::DiffGeometryQuadraticCost(const VarVector& vars, const string& cost_name, double coeff, NeedleProblemHelperPtr helper) : Cost(cost_name), vars(vars), coeff(coeff), helper(helper) {
    for (int i = 0; i < vars.size(); ++i) {
      exprInc(expr, exprMult(exprSquare(vars[i]), coeff));
    }
  }

  double DiffGeometryQuadraticCost::value(const vector<double>& xvec, Model* model) {
    VectorXd vals = getVec(xvec, vars);
    return vals.array().square().sum() * coeff;
  }

  ConvexObjectivePtr DiffGeometryQuadraticCost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    out->addQuadExpr(expr);
    return out;
  }

  DiffGeometryL1Cost::DiffGeometryL1Cost(const VarVector& vars, const string& cost_name, double coeff, NeedleProblemHelperPtr helper) : Cost(cost_name), vars(vars), coeff(coeff), helper(helper) {}

  double DiffGeometryL1Cost::value(const vector<double>& xvec, Model* model) {
    VectorXd vals = getVec(xvec, vars);
    return vals.array().abs().sum() * coeff;
  }

  ConvexObjectivePtr DiffGeometryL1Cost::convex(const vector<double>& xvec) {
    ConvexObjectivePtr out(new ConvexObjective());
    for (int i = 0; i < vars.size(); ++i) {
      out->addAbs(AffExpr(vars[i]), coeff);
    }
    return out;
  }

  NeedleCollisionClearanceCost::NeedleCollisionClearanceCost(NeedleProblemHelperPtr helper, double coeff) :
    Cost("needle_collision_clearance"),
    helper(helper),
    coeff(coeff) {
    this->ccs.clear();
    for (int i = 0; i < helper->pis.size(); ++i) {
      NeedleProblemInstancePtr pi = helper->pis[i];
      for (int j = 0; j < (int) pi->collision_constraints.size(); ++j) {
        ccs.push_back(boost::static_pointer_cast<CollisionConstraint>(pi->collision_constraints[j])->calc());
      }
    }
    for (int i = 0; i < helper->self_collision_constraints.size(); ++i) {
      ccs.push_back(boost::static_pointer_cast<CollisionConstraint>(helper->self_collision_constraints[i])->calc());
    }
  }

  ConvexObjectivePtr NeedleCollisionClearanceCost::convex(const vector<double>& x) {
    ConvexObjectivePtr out(new ConvexObjective());
    vector<AffExpr> exprs;
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    BOOST_FOREACH(const CollisionEvaluatorPtr& cc, this->ccs) {
      vector<AffExpr> tmp_exprs;
      cc->CalcDistExpressions(x, tmp_exprs);
      for (int i = 0; i < tmp_exprs.size(); ++i) {
        exprs.push_back(exprMult(tmp_exprs[i], -1));
      }
    }
    if (exprs.size() > 0) {
      out->addMax(exprs, this->coeff);
    } else {
      out->addAffExpr(AffExpr(-helper->collision_clearance_threshold * this->coeff));
    }
    return out;
  }

  double NeedleCollisionClearanceCost::value(const vector<double>& x, Model* model) {
    DblVec dists;
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    BOOST_FOREACH(const CollisionEvaluatorPtr& cc, this->ccs) {
      DblVec tmp_dists;
      cc->CalcDists(x, tmp_dists);
      for (int i = 0; i < tmp_dists.size(); ++i) {
        dists.push_back(-tmp_dists[i]);
      }
    }
    if (dists.size() > 0) {
      return vecMax(dists) * this->coeff;
    } else {
      return - helper->collision_clearance_threshold * this->coeff;
    }
  }
}
