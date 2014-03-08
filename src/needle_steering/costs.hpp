#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_instance.hpp"

namespace Needle {
  class ConstantSpeedCost : public Cost {
  public:
    ConstantSpeedCost(const Var& var, double coeff, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    virtual double value(const vector<double>& xvec, Model* model);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec);
  private:
    Var var;
    double coeff;
    AffExpr expr;
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
  };

  class RotationQuadraticCost : public Cost {
  public:
    RotationQuadraticCost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper);
    virtual double value(const vector<double>& xvec, Model* model);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec);
  private:
    VarVector vars;
    double coeff;
    QuadExpr expr;
    NeedleProblemHelperPtr helper;
  };

  class RotationL1Cost : public Cost {
  public:
    RotationL1Cost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper);
    virtual double value(const vector<double>& xvec, Model* model);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec);
  private:
    VarVector vars;
    double coeff;
    AffExpr expr;
    NeedleProblemHelperPtr helper;
  };


  class RotationContinuityQuadraticCost : public Cost {
  public:
    RotationContinuityQuadraticCost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper);
    virtual double value(const vector<double>& xvec, Model* model);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec);
  private:
    VarVector vars;
    double coeff;
    QuadExpr expr;
    NeedleProblemHelperPtr helper;
  };

  class RotationContinuityL1Cost : public Cost {
  public:
    RotationContinuityL1Cost(const VarVector& vars, double coeff, NeedleProblemHelperPtr helper);
    virtual double value(const vector<double>& xvec, Model* model);
    virtual ConvexObjectivePtr convex(const vector<double>& xvec);
  private:
    VarVector vars;
    double coeff;
    AffExpr expr;
    NeedleProblemHelperPtr helper;
  };


  struct NeedleCollisionClearanceCost : public Cost {
    NeedleCollisionClearanceCost(NeedleProblemHelperPtr helper, double coeff);
    ConvexObjectivePtr convex(const vector<double>& x);
    double value(const vector<double>& x, Model* model);

    NeedleProblemHelperPtr helper;
    double coeff;
    vector<CollisionEvaluatorPtr> ccs;
  };
}
