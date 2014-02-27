#pragma once

#include "common.hpp"
#include "fwd.hpp"

namespace Needle {
  struct SquarePositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    Vector3d position_error_relax;
    double orientation_error_relax;
    NeedleProblemHelperPtr helper;
    SquarePositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, const Vector3d& position_error_relax, double orientation_error_relax, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct CirclePositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    Vector3d position_error_relax;
    double orientation_error_relax;
    NeedleProblemHelperPtr helper;
    CirclePositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, const Vector3d& position_error_relax, double orientation_error_relax, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct ExactPositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    NeedleProblemHelperPtr helper;
    ExactPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct BallPositionError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    NeedleProblemHelperPtr helper;
    double distance_error_relax;
    BallPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, double distance_error_relax, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct PoseError : public VectorOfVector {
    LocalConfigurationPtr cfg0;
    LocalConfigurationPtr cfg1;
    NeedleProblemHelperPtr helper;
    PoseError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct SpeedDeviationError : public VectorOfVector {
    NeedleProblemHelperPtr helper;
    double deviation;
    SpeedDeviationError(double deviation, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct ControlError : public VectorOfVector {
    LocalConfigurationPtr cfg0, cfg1;
    KinBodyPtr body;
    NeedleProblemHelperPtr helper;
    ControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };

  struct ChannelSurfaceDistance : public VectorOfVector {
    LocalConfigurationPtr cfg;
    NeedleProblemHelperPtr helper;
    ChannelSurfaceDistance(LocalConfigurationPtr cfg, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct TotalCurvatureError : public VectorOfVector {
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
    double total_curvature_limit;
    TotalCurvatureError(double total_curvature_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct TotalCurvatureCostError : public VectorOfVector {
    boost::shared_ptr<TotalCurvatureError> err;
    TotalCurvatureCostError(double total_curvature_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    VectorXd operator()(const VectorXd& a) const;
  };

}
