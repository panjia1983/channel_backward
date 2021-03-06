#include "optimizers.hpp"
#include "modeling.hpp"
#include "utils/logging.hpp"
#include <boost/foreach.hpp>
#include "solver_interface.hpp"
#include "expr_ops.hpp"
#include <cmath>
#include <cstdio>
#include "sco_common.hpp"
#include "utils/stl_to_string.hpp"
#include "macros.h"
#include <boost/format.hpp>
using namespace std;
using namespace util;

#define INC_LOG_TRUST_REGION trust_region_size_history.push_back(vector<double>()); \
                             log_trust_region_size_history.push_back(vector<double>());

#define LOG_TRUST_REGION trust_region_size_history.back().push_back(trust_box_size_); \
                         log_trust_region_size_history.back().push_back(log(trust_box_size_));

#define CHECK_STATUS(status, model) {                                                                                    \
  if (status != CVX_SOLVED) {                                                                                            \
    LOG_ERROR("convex solver failed! set TRAJOPT_LOG_THRESH=DEBUG to see solver output. saving model to /tmp/fail2.lp"); \
    model->writeToFile("/tmp/fail2.lp");                                                                                 \
    retval = OPT_FAILED;                                                                                                 \
    goto cleanup;                                                                                                        \
  }                                                                                                                      \
}

#define RESOLVE_QP() {                                                                                            \
  BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->removeFromModel(model_.get());                   \
  QuadExpr objective;                                                                                             \
  cnt_cost_models = cntsToCosts(cnt_models, merit_error_coeff_);                                                  \
  BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->addToModelAndObjective(model_.get(), objective); \
  model_->setObjective(objective);                                                                                \
  CvxOptStatus status = model_->optimize();                                                                       \
  ++results_.n_qp_solves;                                                                                         \
  CHECK_STATUS(status, model_);                                                                                   \
  current_model_var_vals = model_->getVarValues(model_->getVars());                                               \
  current_model_cnt_viols = evaluateModelCntViols(cnt_models, current_model_var_vals, model_.get());              \
  current_model_total_cnt_viol = vecSum(current_model_cnt_viols);                                                 \
}

#define RESOLVE_QP_1() {                                                                                            \
  BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->removeFromModel(model_.get());                   \
  QuadExpr objective;                                                                                             \
  cnt_cost_models = cntsToCosts(cnt_models, merit_error_coeff_);                                                  \
  BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->addToModelAndObjective(model_.get(), objective); \
  model_->setObjective(objective);                                                                                \
  CvxOptStatus status = model_->optimize();                                                                       \
  ++results_.n_qp_solves;                                                                                         \
  CHECK_STATUS(status, model_);                                                                                   \
  model_var_vals = model_->getVarValues(model_->getVars());                                               \
  model_cnt_viols = evaluateModelCntViols(cnt_models, model_var_vals, model_.get());              \
  current_model_total_cnt_viol = vecSum(model_cnt_viols);                                                 \
}
namespace sco {

typedef vector<double> DblVec;

std::ostream& operator<<(std::ostream& o, const OptResults& r) {
  o << "Optimization results:" << endl
    << "status: " << statusToString(r.status) << endl
    << "cost values: " << Str(r.cost_vals) << endl
    << "constraint violations: " << Str(r.cnt_viols) << endl
    << "n func evals: " << r.n_func_evals << endl
    << "n qp solves: " << r.n_qp_solves << endl
    << "n lp solves: " << r.n_lp_solves << endl
    << "n merit increases: " << r.n_merit_increases << endl;
  o  << "variable values: ";
  for (int i = 0; i < r.x.size(); ++i) {
    o << r.x[i];
    if (i < r.x.size() - 1) {
      o << ", ";
    } else {
      o << endl;
    }
  }
  return o;
}

//////////////////////////////////////////////////
////////// private utility functions for  sqp /////////
//////////////////////////////////////////////////



DblVec evaluateCosts(vector<CostPtr>& costs, const DblVec& x, Model* model) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x, model);
  }
  return out;
}
DblVec evaluateConstraintViols(vector<ConstraintPtr>& constraints, const DblVec& x, Model* model) {
  DblVec out(constraints.size());
  for (size_t i=0; i < constraints.size(); ++i) {
    out[i] = constraints[i]->violation(x, model);
  }
  return out;
}
vector<ConvexObjectivePtr> convexifyCosts(vector<CostPtr>& costs, const DblVec& x) {
  vector<ConvexObjectivePtr> out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->convex(x);
  }
  return out;
}

vector<ConvexConstraintsPtr> convexifyConstraints(vector<ConstraintPtr>& cnts, const DblVec& x) {
  vector<ConvexConstraintsPtr> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->convex(x);
  }
  return out;
}

DblVec evaluateModelCosts(vector<ConvexObjectivePtr>& costs, const DblVec& x, Model* model) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x, model);
  }
  return out;
}
DblVec evaluateModelCntViols(vector<ConvexConstraintsPtr>& cnts, const DblVec& x, Model* model) {
  DblVec out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->violation(x, model);
  }
  return out;
}

vector<string> getCostNames(const vector<CostPtr>& costs) {
  vector<string> out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) out[i] = costs[i]->name();
  return out;
}
vector<string> getCntNames(const vector<ConstraintPtr>& cnts) {
  vector<string> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) out[i] = cnts[i]->name();
  return out;
}

void printCostInfo(const vector<double>& old_cost_vals, const vector<double>& model_cost_vals, const vector<double>& new_cost_vals,
                  const vector<double>& old_cnt_vals, const vector<double>& model_cnt_vals, const vector<double>& new_cnt_vals,
    const vector<string>& cost_names, const vector<string>& cnt_names, double merit_coeff) {
    printf("%15s | %10s | %10s | %10s | %10s\n", "", "oldexact", "newexact", "dapprox", "dexact", "ratio");
    printf("%15s | %10s---%10s---%10s---%10s\n", "COSTS", "----------", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cost_vals.size(); ++i) {
      double approx_improve = old_cost_vals[i] - model_cost_vals[i];
      double exact_improve = old_cost_vals[i] - new_cost_vals[i];
      if (fabs(approx_improve) > 1e-8) 
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e\n", cost_names[i].c_str(), old_cost_vals[i], new_cost_vals[i], approx_improve, exact_improve, exact_improve/approx_improve);
      else
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e | %10s\n",cost_names[i].c_str(), old_cost_vals[i], new_cost_vals[i], approx_improve, exact_improve, "  ------  ");
    }
    if (cnt_names.size() == 0) return;
    printf("%15s | %10s---%10s---%10s---%10s\n", "CONSTRAINTS", "----------", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cnt_vals.size(); ++i) {
      if (cnt_names[i].find("collision") != std::string::npos && new_cnt_vals[i] < 1e-8 && old_cnt_vals[i] < 1e-8) {
        continue;
      }
      double approx_improve = old_cnt_vals[i] - model_cnt_vals[i];
      double exact_improve = old_cnt_vals[i] - new_cnt_vals[i];
      if (fabs(approx_improve) > 1e-8)
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e | %10.3e\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], new_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, exact_improve/approx_improve);
      else 
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e | %10s\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], new_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, "  ------  ");
    }

}

// todo: use different coeffs for each constraint
vector<ConvexObjectivePtr> cntsToCosts(const vector<ConvexConstraintsPtr>& cnts, double err_coeff) {
  vector<ConvexObjectivePtr> out;
  BOOST_FOREACH(const ConvexConstraintsPtr& cnt, cnts) {
    ConvexObjectivePtr obj(new ConvexObjective());
    BOOST_FOREACH(const AffExpr& aff, cnt->eqs_) {
      obj->addAbs(aff, err_coeff);
    }
    BOOST_FOREACH(const AffExpr& aff, cnt->ineqs_) {
      obj->addHinge(aff, err_coeff);
      //obj->addL2Norm(aff);
    }
    out.push_back(obj);
  }
  return out;
}

void Optimizer::addCallback(const Callback& cb) {
  callbacks_.push_back(cb);
}

bool Optimizer::callCallbacks(DblVec& x) {
  bool should_recompute = false;
  for (int i=0; i < callbacks_.size(); ++i) {
    bool result = callbacks_[i](prob_.get(), x);
    should_recompute = should_recompute || result;
  }
  return should_recompute;
}

void Optimizer::initialize(const vector<double>& x) {
  if (!prob_) PRINT_AND_THROW("need to set the problem before initializing");
  if (prob_->getVars().size() != x.size()) 
    PRINT_AND_THROW(boost::format("initialization vector has wrong length. expected %i got %i")%prob_->getVars().size()%x.size());
  results_.clear(); 
  results_.x = x;
}

BasicTrustRegionSQP::BasicTrustRegionSQP() {
  initParameters();
}
BasicTrustRegionSQP::BasicTrustRegionSQP(OptProbPtr prob) {
  initParameters();
  setProblem(prob);
}

void BasicTrustRegionSQP::initParameters() {

  improve_ratio_threshold_ = .25;
  min_trust_box_size_ = 1e-4;
  min_approx_improve_= 1e-4;
  min_approx_improve_frac_ = -INFINITY;
  max_iter_ = 50;
  trust_shrink_ratio_=.1;
  trust_expand_ratio_ = 1.5;
  cnt_tolerance_ = 1e-4;
  max_merit_coeff_increases_ = 10;
  merit_coeff_increase_ratio_ = 10;
  max_time_ = INFINITY;
  merit_error_coeff_ = 10;
  trust_box_size_ = 1e-1;
  record_trust_region_history_ = false;

}

void BasicTrustRegionSQP::setProblem(OptProbPtr prob) {
  Optimizer::setProblem(prob);
  model_ = prob->getModel();
}

void BasicTrustRegionSQP::adjustTrustRegion(double ratio) {
  trust_box_size_ = fmin(trust_box_size_ * ratio, 100);
}

bool BasicTrustRegionSQP::hasViolation(const DblVec& cnt_viols) {
  return cnt_viols.size() > 0 && vecMax(cnt_viols) > cnt_tolerance_;
}

void BasicTrustRegionSQP::setTrustBoxConstraints(const DblVec& x, Model* model) {
  assert (model != NULL);
  vector<Var>& vars = prob_->getVars();
  assert(vars.size() == x.size());
  DblVec& lb=prob_->getLowerBounds(), ub=prob_->getUpperBounds();
  DblVec lbtrust(x.size()), ubtrust(x.size());
  for (size_t i=0; i < x.size(); ++i) {
    lbtrust[i] = fmax(x[i] - trust_box_size_, lb[i]);
    ubtrust[i] = fmin(x[i] + trust_box_size_, ub[i]);
  }
  model->setVarBounds(vars, lbtrust, ubtrust);
}

#if 0
struct MultiCritFilter {
  /**
   * Checks if you're making an improvement on a multidimensional objective
   * Given a set of past error vectors, the improvement is defined as
   * min_{olderrvec in past_err_vecs} | olderrvec - errvec |^+
   */
  vector<DblVec> errvecs;
  double improvement(const DblVec& errvec) {
    double leastImprovement=INFINITY;
    BOOST_FOREACH(const DblVec& olderrvec, errvecs) {
      double improvement=0;
      for (int i=0; i < errvec.size(); ++i) improvement += pospart(olderrvec[i] - errvec[i]);
      leastImprovement = fmin(leastImprovement, improvement);
    }
    return leastImprovement;
  }
  void insert(const DblVec& x) {errvecs.push_back(x);}
  bool empty() {return errvecs.size() > 0;}
};
#endif

OptStatus BasicTrustRegionSQP::optimize() {

  vector<string> cost_names = getCostNames(prob_->getCosts());
  vector<ConstraintPtr> constraints = prob_->getConstraints();
  vector<string> cnt_names = getCntNames(constraints);

  DblVec& x_ = results_.x; // just so I don't have to rewrite code
  if (x_.size() == 0) PRINT_AND_THROW("you forgot to initialize!");
  if (!prob_) PRINT_AND_THROW("you forgot to set the optimization problem");


  // cache the latest feasible result;
  DblVec cached_feasible_x;

  
  x_ = prob_->getClosestFeasiblePoint(x_);

  assert(x_.size() == prob_->getVars().size());
  assert(prob_->getCosts().size() > 0 || constraints.size() > 0);

  OptStatus retval = INVALID;

  if (record_trust_region_history_) {
    INC_LOG_TRUST_REGION;
    LOG_TRUST_REGION;
  }

  for (int merit_increases=0; merit_increases < max_merit_coeff_increases_; ) { /* merit adjustment loop */
    //++results_.n_merit_increases;

    for (int iter=1; ; ++iter) { /* sqp loop */
      bool should_recompute = callCallbacks(x_);

      LOG_DEBUG("current iterate: %s", CSTR(x_));
      LOG_INFO("merit increases iteration: %i; sqp iteration %i", merit_increases, iter);

      // speedup: if you just evaluated the cost when doing the line search, use that
      if (results_.cost_vals.empty() && results_.cnt_viols.empty() || should_recompute) { //only happens on the first iteration
        results_.cnt_viols = evaluateConstraintViols(constraints, x_, model_.get());
        results_.cost_vals = evaluateCosts(prob_->getCosts(), x_, model_.get());
        assert(results_.n_func_evals == 0 || should_recompute);
        ++results_.n_func_evals;
      }

      // DblVec new_cnt_viols = evaluateConstraintViols(constraints, x_);
      // DblVec new_cost_vals = evaluateCosts(prob_->getCosts(), x_);
      // cout << "costs" << endl;
      // for (int i=0; i < new_cnt_viols.size(); ++i) {
      //   cout << cnt_names[i] << " " << new_cnt_viols[i] - results_.cnt_viols[i] << endl;
      // }
      // for (int i=0; i < new_cost_vals.size(); ++i) {
      //   cout << cost_names[i] << " " << new_cost_vals[i] - results_.cost_vals[i] << endl;
      // }


      vector<ConvexObjectivePtr> cost_models = convexifyCosts(prob_->getCosts(),x_);
      vector<ConvexConstraintsPtr> cnt_models = convexifyConstraints(constraints, x_);
      vector<ConvexObjectivePtr> cnt_cost_models = cntsToCosts(cnt_models, merit_error_coeff_);

      QuadExpr objective;

      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) cost->addToModelAndObjective(model_.get(), objective);
      BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->addToModelAndObjective(model_.get(), objective);
      //model_->update();

      //BOOST_FOREACH(ConvexObjectivePtr& co, cost_models) exprInc(objective, co->quad_);
      //BOOST_FOREACH(ConvexObjectivePtr& co, cnt_cost_models){
      //  exprInc(objective, co->quad_);
      //}
//    objective = cleanupExpr(objective);
      model_->setObjective(objective);

//    if (logging::filter() >= IPI_LEVEL_DEBUG) {
//      DblVec model_cost_vals;
//      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) {
//        model_cost_vals.push_back(cost->value(x));
//      }
//      LOG_DEBUG("model costs %s should equalcosts  %s", printer(model_cost_vals), printer(cost_vals));
//    }
//
      DblVec old_model_var_vals = x_;//model_->getVarValues(model_->getVars());
      for (int i = 0; i < model_->getVars().size() - x_.size(); ++i) {
        old_model_var_vals.push_back(0.); 
      }

      DblVec model_var_vals;
      DblVec model_cost_vals;
      DblVec model_cnt_viols;
      DblVec new_cost_vals;
      DblVec new_cnt_viols;
      DblVec new_x;

      while (trust_box_size_ >= min_trust_box_size_) {

        setTrustBoxConstraints(x_, model_.get());
        CvxOptStatus status = model_->optimize();
        ++results_.n_qp_solves;
        if (status != CVX_SOLVED) {
          LOG_ERROR("convex solver failed! set TRAJOPT_LOG_THRESH=DEBUG to see solver output. saving model to /tmp/fail2.lp");
          model_->writeToFile("/tmp/fail2.lp");
          retval = OPT_FAILED;
          goto cleanup;
        }
        model_var_vals = model_->getVarValues(model_->getVars());

        model_cost_vals = evaluateModelCosts(cost_models, model_var_vals, model_.get());
        model_cnt_viols = evaluateModelCntViols(cnt_models, model_var_vals, model_.get());

        // the n variables of the OptProb happen to be the first n variables in the Model
        new_x = DblVec(model_var_vals.begin(), model_var_vals.begin() + x_.size());


        // cout << "new_x" << endl;
        // for (int i = 0; i < new_x.size(); ++i)
        //   cout << new_x[i] << " ";
        // cout << endl;

        if (GetLogLevel() >= util::LevelDebug) {
          DblVec cnt_costs1 = evaluateModelCosts(cnt_cost_models, model_var_vals, model_.get());
          DblVec cnt_costs2 = model_cnt_viols;
          for (int i=0; i < cnt_costs2.size(); ++i) cnt_costs2[i] *= merit_error_coeff_;
          LOG_DEBUG("SHOULD BE ALMOST THE SAME: %s ?= %s", CSTR(cnt_costs1), CSTR(cnt_costs2) );
          // not exactly the same because cnt_costs1 is based on aux variables, but they might not be at EXACTLY the right value
        }

        new_cost_vals = evaluateCosts(prob_->getCosts(), new_x, model_.get());
        new_cnt_viols = evaluateConstraintViols(constraints, new_x, model_.get());
        ++results_.n_func_evals;

        double old_merit = vecSum(results_.cost_vals) + merit_error_coeff_ * vecSum(results_.cnt_viols);
        double model_merit = vecSum(model_cost_vals) + merit_error_coeff_ * vecSum(model_cnt_viols);
        double new_merit = vecSum(new_cost_vals) + merit_error_coeff_ * vecSum(new_cnt_viols);
        double approx_merit_improve = old_merit - model_merit;
        double exact_merit_improve = old_merit - new_merit;
        double merit_improve_ratio = exact_merit_improve / approx_merit_improve;

        if (util::GetLogLevel() >= util::LevelInfo) {
          LOG_INFO(" ");
          printCostInfo(results_.cost_vals, model_cost_vals, new_cost_vals,
                        results_.cnt_viols, model_cnt_viols, new_cnt_viols, cost_names,
                        cnt_names, merit_error_coeff_);
          printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", "TOTAL", old_merit, approx_merit_improve, exact_merit_improve, merit_improve_ratio);
        }

        if (approx_merit_improve < -1e-5) {
          LOG_ERROR("approximate merit function got worse (%.3e). (convexification is probably wrong to zeroth order)", approx_merit_improve);
        }
        if (approx_merit_improve < min_approx_improve_) {
          LOG_INFO("converged because improvement was small (%.3e < %.3e)", approx_merit_improve, min_approx_improve_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        }
        if (approx_merit_improve / old_merit < min_approx_improve_frac_) {
          LOG_INFO(
              "converged because improvement ratio was small (%.3e < %.3e)",
              approx_merit_improve/old_merit, min_approx_improve_frac_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        } 
        else if (exact_merit_improve < 0 || merit_improve_ratio < improve_ratio_threshold_) {
          adjustTrustRegion(trust_shrink_ratio_);
          LOG_INFO("shrunk trust region. new box size: %.4f",
              trust_box_size_);
          if (record_trust_region_history_) {
            LOG_TRUST_REGION;
          }
        } else {
          x_ = new_x;
          results_.cost_vals = new_cost_vals;
          results_.cnt_viols = new_cnt_viols;
          adjustTrustRegion(trust_expand_ratio_);
          LOG_INFO("expanded trust region. new box size: %.4f",trust_box_size_);
          if (record_trust_region_history_) {
            LOG_TRUST_REGION;
          }
          break;
        }
      }

      if (trust_box_size_ < min_trust_box_size_) {
        LOG_INFO("converged because trust region is tiny");
        retval = OPT_CONVERGED;
        goto penaltyadjustment;
      } else if (iter >= max_iter_) {
        LOG_INFO("iteration limit");
        retval = OPT_SCO_ITERATION_LIMIT;
        goto cleanup;
      }

      continue;

      penaltyadjustment:
      if (results_.cnt_viols.empty() || vecMax(results_.cnt_viols) < cnt_tolerance_) {
        if (results_.cnt_viols.size() > 0) LOG_INFO("woo-hoo! constraints are satisfied (to tolerance %.2e)", cnt_tolerance_);
        goto cleanup;
      }
      else {
        LOG_INFO("not all constraints are satisfied. increasing penalties");
        merit_error_coeff_ *= merit_coeff_increase_ratio_;
        ++merit_increases;
        ++results_.n_merit_increases;
        trust_box_size_ = fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5);
        if (record_trust_region_history_) {
          INC_LOG_TRUST_REGION;
          LOG_TRUST_REGION;
        }
        break;
      }
    }
  }
  retval = OPT_PENALTY_ITERATION_LIMIT;
  LOG_INFO("optimization couldn't satisfy all constraints");


  cleanup:
  assert(retval != INVALID && "should never happen");
  results_.status = retval;
  results_.total_cost = vecSum(results_.cost_vals);
  LOG_INFO("\n==================\n%s==================", CSTR(results_));
  callCallbacks(x_);

  return retval;

}

template<typename T> vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> c;
  c.insert(c.end(), a.begin(), a.end());
  c.insert(c.end(), b.begin(), b.end());
  return c;
}

OptStatus NeedleSQP::optimize() {

  vector<string> cost_names = getCostNames(prob_->getCosts());
  vector<ConstraintPtr> constraints = prob_->getConstraints();
  vector<string> cnt_names = getCntNames(constraints);

  vector<ConstraintPtr> dynamics_constraints;
  vector<ConstraintPtr> collision_constraints;

  vector<string> dynamics_constraints_names;
  vector<string> collision_constraints_names;

  for (int i = 0; i < constraints.size(); ++i) {
    if (constraints[i]->name().find("collision") != std::string::npos) {
      collision_constraints.push_back(constraints[i]);
      collision_constraints_names.push_back(constraints[i]->name());
    } else {
      dynamics_constraints.push_back(constraints[i]);
      dynamics_constraints_names.push_back(constraints[i]->name());
    }
  }

  DblVec& x_ = results_.x; // just so I don't have to rewrite code
  if (x_.size() == 0) PRINT_AND_THROW("you forgot to initialize!");
  if (!prob_) PRINT_AND_THROW("you forgot to set the optimization problem");    
  
  x_ = prob_->getClosestFeasiblePoint(x_);

  assert(x_.size() == prob_->getVars().size());
  assert(prob_->getCosts().size() > 0 || constraints.size() > 0);

  double dynamics_merit_error_coeff_ = merit_error_coeff_;
  double collision_merit_error_coeff_ = merit_error_coeff_;

  //cout << "dynamics merit error coeff: " << dynamics_merit_error_coeff_ << endl;
  //cout << "collision merit error coeff: " << collision_merit_error_coeff_ << endl;


  OptStatus retval = INVALID;

  if (record_trust_region_history_) {
    INC_LOG_TRUST_REGION;
    LOG_TRUST_REGION;
  }

  for (int merit_increases=0; merit_increases < max_merit_coeff_increases_; ) { /* merit adjustment loop */
    //++results_.n_merit_increases;

    for (int iter=1; ; ++iter) { /* sqp loop */
      bool should_recompute = callCallbacks(x_);

      LOG_DEBUG("current iterate: %s", CSTR(x_));
      LOG_INFO("merit increases iteration: %i; sqp iteration %i", merit_increases, iter);

      // speedup: if you just evaluated the cost when doing the line search, use that
      if (results_.cost_vals.empty() && results_.dynamics_cnt_viols.empty() && results_.collision_cnt_viols.empty() || should_recompute) { //only happens on the first iteration
        results_.dynamics_cnt_viols = evaluateConstraintViols(dynamics_constraints, x_, model_.get());
        results_.collision_cnt_viols = evaluateConstraintViols(collision_constraints, x_, model_.get());
        results_.cnt_viols = concat(results_.dynamics_cnt_viols, results_.collision_cnt_viols);
        results_.cost_vals = evaluateCosts(prob_->getCosts(), x_, model_.get());
        assert(results_.n_func_evals == 0 || should_recompute);
        ++results_.n_func_evals;
      }


      // DblVec new_cnt_viols = evaluateConstraintViols(constraints, x_);
      // DblVec new_cost_vals = evaluateCosts(prob_->getCosts(), x_);
      // cout << "costs" << endl;
      // for (int i=0; i < new_cnt_viols.size(); ++i) {
      //   cout << cnt_names[i] << " " << new_cnt_viols[i] - results_.cnt_viols[i] << endl;
      // }
      // for (int i=0; i < new_cost_vals.size(); ++i) {
      //   cout << cost_names[i] << " " << new_cost_vals[i] - results_.cost_vals[i] << endl;
      // }


      vector<ConvexObjectivePtr> cost_models = convexifyCosts(prob_->getCosts(),x_);
      vector<ConvexConstraintsPtr> dynamics_cnt_models = convexifyConstraints(dynamics_constraints, x_);
      vector<ConvexConstraintsPtr> collision_cnt_models = convexifyConstraints(collision_constraints, x_);
      vector<ConvexObjectivePtr> dynamics_cnt_cost_models = cntsToCosts(dynamics_cnt_models, dynamics_merit_error_coeff_);
      vector<ConvexObjectivePtr> collision_cnt_cost_models = cntsToCosts(collision_cnt_models, collision_merit_error_coeff_);

      QuadExpr objective;

      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) cost->addToModelAndObjective(model_.get(), objective);
      BOOST_FOREACH(ConvexObjectivePtr& cost, dynamics_cnt_cost_models) cost->addToModelAndObjective(model_.get(), objective);
      BOOST_FOREACH(ConvexObjectivePtr& cost, collision_cnt_cost_models) cost->addToModelAndObjective(model_.get(), objective);
      //model_->update();

      //BOOST_FOREACH(ConvexObjectivePtr& co, cost_models) exprInc(objective, co->quad_);
      //BOOST_FOREACH(ConvexObjectivePtr& co, cnt_cost_models){
      //  exprInc(objective, co->quad_);
      //}
//    objective = cleanupExpr(objective);
      model_->setObjective(objective);

//    if (logging::filter() >= IPI_LEVEL_DEBUG) {
//      DblVec model_cost_vals;
//      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) {
//        model_cost_vals.push_back(cost->value(x));
//      }
//      LOG_DEBUG("model costs %s should equalcosts  %s", printer(model_cost_vals), printer(cost_vals));
//    }
//
      DblVec old_model_var_vals = x_;//model_->getVarValues(model_->getVars());
      for (int i = 0; i < model_->getVars().size() - x_.size(); ++i) {
        old_model_var_vals.push_back(0.); 
      }

      DblVec model_var_vals;
      DblVec model_cost_vals;
      DblVec model_dynamics_cnt_viols;
      DblVec model_collision_cnt_viols;
      DblVec new_cost_vals;
      DblVec new_dynamics_cnt_viols;
      DblVec new_collision_cnt_viols;
      DblVec new_x;

      while (trust_box_size_ >= min_trust_box_size_) {

        //cout << "setting trust box size " << trust_box_size_ << " around: ";
        //for (int i = 0; i < x_.size(); ++i) cout << x_[i] << " ";
        //cout << endl;

        //cout << "current dynamics penalty coefficient: " << dynamics_merit_error_coeff_ << endl;
        //cout << "current collision penalty coefficient: " << collision_merit_error_coeff_ << endl;

        setTrustBoxConstraints(x_, model_.get());
        CvxOptStatus status = model_->optimize();
        ++results_.n_qp_solves;
        if (status != CVX_SOLVED) {
          LOG_ERROR("convex solver failed! set TRAJOPT_LOG_THRESH=DEBUG to see solver output. saving model to /tmp/fail2.lp");
          model_->writeToFile("/tmp/fail2.lp");
          retval = OPT_FAILED;
          goto cleanup;
        }
        model_var_vals = model_->getVarValues(model_->getVars());

        model_cost_vals = evaluateModelCosts(cost_models, model_var_vals, model_.get());
        model_dynamics_cnt_viols = evaluateModelCntViols(dynamics_cnt_models, model_var_vals, model_.get());
        model_collision_cnt_viols = evaluateModelCntViols(collision_cnt_models, model_var_vals, model_.get());

        // the n variables of the OptProb happen to be the first n variables in the Model
        new_x = DblVec(model_var_vals.begin(), model_var_vals.begin() + x_.size());

        //cout << "solution: ";
        //for (int i = 0; i < new_x.size(); ++i) cout << new_x[i] << " ";
        //cout << endl;

        if (GetLogLevel() >= util::LevelDebug) {
          DblVec dynamics_cnt_costs1 = evaluateModelCosts(dynamics_cnt_cost_models, model_var_vals, model_.get());
          DblVec dynamics_cnt_costs2 = model_dynamics_cnt_viols;
          for (int i=0; i < dynamics_cnt_costs2.size(); ++i) dynamics_cnt_costs2[i] *= dynamics_merit_error_coeff_;
          DblVec collision_cnt_costs1 = evaluateModelCosts(collision_cnt_cost_models, model_var_vals, model_.get());
          DblVec collision_cnt_costs2 = model_collision_cnt_viols;
          for (int i=0; i < collision_cnt_costs2.size(); ++i) collision_cnt_costs2[i] *= collision_merit_error_coeff_;


          LOG_DEBUG("SHOULD BE ALMOST THE SAME: %s ?= %s", CSTR(concat(dynamics_cnt_costs1, collision_cnt_costs1)), CSTR(concat(dynamics_cnt_costs2, collision_cnt_costs2)) );
          // not exactly the same because cnt_costs1 is based on aux variables, but they might not be at EXACTLY the right value
        }

        new_cost_vals = evaluateCosts(prob_->getCosts(), new_x, model_.get());
        new_dynamics_cnt_viols = evaluateConstraintViols(dynamics_constraints, new_x, model_.get());
        new_collision_cnt_viols = evaluateConstraintViols(collision_constraints, new_x, model_.get());
        ++results_.n_func_evals;

        double old_merit = vecSum(results_.cost_vals) + dynamics_merit_error_coeff_ * vecSum(results_.dynamics_cnt_viols) + collision_merit_error_coeff_ * vecSum(results_.collision_cnt_viols);
        double model_merit = vecSum(model_cost_vals) + dynamics_merit_error_coeff_ * vecSum(model_dynamics_cnt_viols) + collision_merit_error_coeff_ * vecSum(model_collision_cnt_viols);
        double new_merit = vecSum(new_cost_vals) + dynamics_merit_error_coeff_ * vecSum(new_dynamics_cnt_viols) + collision_merit_error_coeff_ * vecSum(new_collision_cnt_viols);
        double approx_merit_improve = old_merit - model_merit;
        double exact_merit_improve = old_merit - new_merit;
        double merit_improve_ratio = exact_merit_improve / approx_merit_improve;

        if (util::GetLogLevel() >= util::LevelInfo) {
          LOG_INFO(" ");
          printCostInfo(results_.cost_vals, model_cost_vals, new_cost_vals,
                        results_.cnt_viols, concat(model_dynamics_cnt_viols, model_collision_cnt_viols), concat(new_dynamics_cnt_viols, new_collision_cnt_viols), cost_names,
                        concat(dynamics_constraints_names, collision_constraints_names), merit_error_coeff_);
          printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", "TOTAL", old_merit, approx_merit_improve, exact_merit_improve, merit_improve_ratio);
        }

        if (approx_merit_improve < -1e-5) {
          LOG_ERROR("approximate merit function got worse (%.3e). (convexification is probably wrong to zeroth order)", approx_merit_improve);
        }
        if (approx_merit_improve < min_approx_improve_) {
          LOG_INFO("converged because improvement was small (%.3e < %.3e)", approx_merit_improve, min_approx_improve_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        }
        if (approx_merit_improve / old_merit < min_approx_improve_frac_) {
          LOG_INFO(
              "converged because improvement ratio was small (%.3e < %.3e)",
              approx_merit_improve/old_merit, min_approx_improve_frac_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        } 
        else if (exact_merit_improve < 0 || merit_improve_ratio < improve_ratio_threshold_) {
          adjustTrustRegion(trust_shrink_ratio_);
          LOG_INFO("shrunk trust region. new box size: %.4f",
              trust_box_size_);
          if (record_trust_region_history_) {
            LOG_TRUST_REGION;
          }
        } else {
          x_ = new_x;
          //cout << "adopting new_x: ";
          //for (int i = 0; i < x_.size(); ++i) { cout << x_[i] << " "; } cout << endl;
          results_.cost_vals = new_cost_vals;
          results_.dynamics_cnt_viols = new_dynamics_cnt_viols;
          results_.collision_cnt_viols = new_collision_cnt_viols;
          results_.cnt_viols = concat(results_.dynamics_cnt_viols, results_.collision_cnt_viols);
          
          adjustTrustRegion(trust_expand_ratio_);
          LOG_INFO("expanded trust region. new box size: %.4f",trust_box_size_);
          if (record_trust_region_history_) {
            LOG_TRUST_REGION;
          }
          break;
        }
      }

      if (trust_box_size_ < min_trust_box_size_) {
        LOG_INFO("converged because trust region is tiny");
        retval = OPT_CONVERGED;
        goto penaltyadjustment;
      } else if (iter >= max_iter_) {
        LOG_INFO("iteration limit");
        retval = OPT_SCO_ITERATION_LIMIT;
        goto cleanup;
      }

      continue;

      penaltyadjustment:
      if (results_.cnt_viols.empty() || vecMax(results_.cnt_viols) < cnt_tolerance_) {
        if (results_.cnt_viols.size() > 0) LOG_INFO("woo-hoo! constraints are satisfied (to tolerance %.2e)", cnt_tolerance_);
        goto cleanup;
      }
      else {
        LOG_INFO("not all constraints are satisfied. increasing penalties");
        //++merit_increases;
        
        if (results_.dynamics_cnt_viols.empty() || vecMax(results_.dynamics_cnt_viols) < cnt_tolerance_) {
          ++merit_increases;
          ++results_.n_merit_increases;
          collision_merit_error_coeff_ *= merit_coeff_increase_ratio_;
          dynamics_merit_error_coeff_ *= merit_coeff_increase_ratio_;
          LOG_INFO("increasing both");
          //cout << "increasing just collision" << endl;
          trust_box_size_ = fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5);

        } else {
          ++merit_increases;
          ++results_.n_merit_increases;
          dynamics_merit_error_coeff_ *= merit_coeff_increase_ratio_;
          LOG_INFO("increasing just dynamics");
          trust_box_size_ = fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5);
        }

        LOG_INFO("(%f, %f)", collision_merit_error_coeff_, dynamics_merit_error_coeff_);

        if (record_trust_region_history_) {
          INC_LOG_TRUST_REGION;
          LOG_TRUST_REGION;
        }
        //callCallbacks(x_);
        break;
      }
    }
  }
  retval = OPT_PENALTY_ITERATION_LIMIT;
  LOG_INFO("optimization couldn't satisfy all constraints");


  cleanup:
  assert(retval != INVALID && "should never happen");
  results_.status = retval;
  results_.total_cost = vecSum(results_.cost_vals);
  LOG_INFO("\n==================\n%s==================", CSTR(results_));
  callCallbacks(x_);

  return retval;

}

}
