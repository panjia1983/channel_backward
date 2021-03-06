#pragma once
#include "trajopt/common.hpp"
#include "trajopt/collision_checker.hpp"
#include "utils/default_map.hpp"
#include "sco/modeling.hpp"
#include "sco/sco_fwd.hpp"
#include "cache.hxx"


namespace trajopt {

typedef std::map<const OR::KinBody::Link*, int> Link2Int;

struct TRAJOPT_API CollisionHash : OpenRAVE::UserData {
  virtual size_t hash(const DblVec& x) = 0;
  static boost::shared_ptr<CollisionHash> Get(OpenRAVE::EnvironmentBase& env);
};

typedef boost::shared_ptr<CollisionHash> CollisionHashPtr;

struct TRAJOPT_API CollisionEvaluator {
  virtual void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, vector<Collision>& collisions) = 0;
  virtual void GetCollisionsCached(const DblVec& x, vector<Collision>&);
  virtual void CollisionsToDistances(const vector<Collision>& collisions, const Link2Int& m_link2ind,
    DblVec& dists);
  virtual void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad,
    const Link2Int& link2ind, const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1);
  virtual void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad0, Configuration& rad1, const Link2Int& link2ind,
    const VarVector& vars0, const VarVector& vars1, const DblVec& vals0, const DblVec& vals1,
    vector<AffExpr>& exprs);
  virtual void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad00, Configuration& rad01, Configuration& rad10, Configuration& rad11, const Link2Int& link2ind0, const Link2Int& link2ind1,
    const VarVector& vars00, const VarVector& vars01, const VarVector& vars10, const VarVector& vars11,
    const DblVec& vals00, const DblVec& vals01, const DblVec& vals10, const DblVec& vals11,
    vector<AffExpr>& exprs);
  virtual size_t hash(const DblVec& x);
  virtual ~CollisionEvaluator() {}
  virtual VarVector GetVars()=0;
  void SetCollisionHash(CollisionHashPtr cc_hash) { m_cc_hash = cc_hash; }

  CollisionHashPtr m_cc_hash;
  Cache<size_t, vector<Collision>, 3> m_cache;
};

typedef boost::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct TRAJOPT_API SingleTimestepCollisionEvaluator : public CollisionEvaluator {
public:
  SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance function
  */
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs); 
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs); 
  void CalcCollisions(const DblVec& x, vector<Collision>& collisions);
  VarVector GetVars() {return m_vars;}

  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarVector m_vars;
  Link2Int m_link2ind;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;
};

struct TRAJOPT_API CastCollisionEvaluator : public CollisionEvaluator {
public:
  CastCollisionEvaluator(ConfigurationPtr rad0, ConfigurationPtr rad1, const VarVector& vars0, const VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, vector<Collision>& collisions);
  VarVector GetVars() {return concat(m_vars0, m_vars1);}
  

  // parameters:
  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad0;
  ConfigurationPtr m_rad1;
  VarVector m_vars0;
  VarVector m_vars1;
  typedef std::map<const OR::KinBody::Link*, int> Link2Int;
  Link2Int m_link2ind;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;

};

struct TRAJOPT_API CastSelfCollisionEvaluator : public CollisionEvaluator {
public:
  CastSelfCollisionEvaluator(
      ConfigurationPtr rad00, ConfigurationPtr rad01, ConfigurationPtr rad10, ConfigurationPtr rad11,
      const VarVector& vars00, const VarVector& vars01, const VarVector& vars10, const VarVector& vars11
  );
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, vector<Collision>& collisions);
  VarVector GetVars() {return concat(m_vars00, concat(m_vars01, concat(m_vars10, m_vars11)));}
  

  // parameters:
  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad00;
  ConfigurationPtr m_rad01;
  ConfigurationPtr m_rad10;
  ConfigurationPtr m_rad11;
  VarVector m_vars00;
  VarVector m_vars01;
  VarVector m_vars10;
  VarVector m_vars11;
  typedef std::map<const OR::KinBody::Link*, int> Link2Int;
  Link2Int m_link2ind0, m_link2ind1;
  vector<OR::KinBody::LinkPtr> m_links0, m_links1;
  short m_filterMask;

};



class TRAJOPT_API CollisionCost : public Cost, public Plotter {
public:
  /* constructor for single timestep */
  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad0, ConfigurationPtr rad1, const VarVector& vars0, const VarVector& vars1);
  /* constructor for cast self collision cost */
  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad00, ConfigurationPtr rad01, ConfigurationPtr rad10, ConfigurationPtr rad11, const VarVector& vars00, const VarVector& vars01, const VarVector& vars10, const VarVector& vars11);
  virtual ConvexObjectivePtr convex(const vector<double>& x);
  virtual double value(const vector<double>&, Model*);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
protected:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};

class TRAJOPT_API CollisionConstraint : public IneqConstraint {
public:
  /* constructor for single timestep */
  CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars);
  /* constructor for cast constraint */
  CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad0, ConfigurationPtr rad1, const VarVector& vars0, const VarVector& vars1);
  /* constructor for cast self collision constraint */
  CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad00, ConfigurationPtr rad01, ConfigurationPtr rad10, ConfigurationPtr rad11, const VarVector& vars00, const VarVector& vars01, const VarVector& vars10, const VarVector& vars11);
  virtual ConvexConstraintsPtr convex(const vector<double>& x);
  virtual DblVec value(const vector<double>&, Model*);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
  CollisionEvaluatorPtr calc() { return m_calc; }
protected:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};

}
