#pragma once

#include "common.hpp"
#include "fwd.hpp"

namespace Needle {
  struct TranslationError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    Vector3d position_error_relax;
    NeedleProblemHelperPtr helper;
    TranslationError(LocalConfigurationPtr cfg, const Vector6d& target_pos, const Vector3d& position_error_relax, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct CircleOrientationError : public VectorOfVector {
    LocalConfigurationPtr cfg;
    KinBodyPtr body;
    Matrix4d target_pose;
    Vector3d position_error_relax;
    double orientation_error_relax;
    NeedleProblemHelperPtr helper;
    CircleOrientationError(LocalConfigurationPtr cfg, const Vector6d& target_pos, double orientation_error_relax, NeedleProblemHelperPtr helper);
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


  struct ControlErrorFirstFixed : public VectorOfVector {
    LocalConfigurationPtr cfg0, cfg1;
    KinBodyPtr body;
    NeedleProblemHelperPtr helper;
    ControlErrorFirstFixed(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };

  struct LinearizedControlError : public VectorOfVector {
    LocalConfigurationPtr cfg0, cfg1;
    KinBodyPtr body;
    NeedleProblemHelperPtr helper;
    LinearizedControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };

  struct LinearizedControlErrorFirstFixed : public VectorOfVector {
    LocalConfigurationPtr cfg0, cfg1;
    KinBodyPtr body;
    NeedleProblemHelperPtr helper;
    LinearizedControlErrorFirstFixed(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };

  struct ChannelSurfaceDistance : public VectorOfVector {
    LocalConfigurationPtr cfg;
    NeedleProblemHelperPtr helper;
    ChannelSurfaceDistance(LocalConfigurationPtr cfg, NeedleProblemHelperPtr helper);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct CurvatureError : public VectorOfVector {
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
    double total_curvature_limit;
    CurvatureError(double total_curvature_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct RotationError : public VectorOfVector {
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
    double total_rotation_limit;
    RotationError(double total_rotation_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    VectorXd operator()(const VectorXd& a) const;
  };

  struct CurvatureCost : public ScalarOfVector {
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
    CurvatureCost(NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    double operator()(const VectorXd& a) const;
  };

  struct TwistError : public VectorOfVector {
    NeedleProblemHelperPtr helper;
    NeedleProblemInstancePtr pi;
    double twist_translation_limit;
    double twist_rotation_limit;
    TwistError(double twist_translation_limit, double twist_rotation_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi);
    VectorXd operator()(const VectorXd& a) const;
    int outputSize() const;
  };


}
