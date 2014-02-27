#pragma once

#include "common.hpp"
#include "fwd.hpp"

namespace Needle {
  struct LocalConfiguration : public Configuration {
    KinBodyPtr body;
    Matrix4d pose;
    LocalConfiguration(KinBodyPtr body, const Matrix4d& pose);
    LocalConfiguration(KinBodyPtr body);
    virtual void SetDOFValues(const DblVec& dofs);
    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const;
    virtual DblVec GetDOFValues() const;
    virtual int GetDOF() const;
    virtual OpenRAVE::EnvironmentBasePtr GetEnv();
    virtual DblMatrix PositionJacobian(int link_ind, const OpenRAVE::Vector& pt) const;
    virtual DblMatrix RotationJacobian(int link_ind) const;
    virtual bool DoesAffect(const KinBody::Link& link);
    virtual std::vector<KinBody::LinkPtr> GetAffectedLinks();
    virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links,
        bool only_with_geom, vector<int>& link_inds);
    virtual DblVec RandomDOFValues();
    virtual vector<OpenRAVE::KinBodyPtr> GetBodies();
  };
}
