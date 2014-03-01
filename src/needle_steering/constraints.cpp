#include "constraints.hpp"
#include "utils.hpp"
#include "local_configuration.hpp"
#include "needle_problem_helper.hpp"
#include <unsupported/Eigen/MatrixFunctions>

namespace Needle {

  TranslationError::TranslationError(LocalConfigurationPtr cfg, const Vector6d& target_pos, const Vector3d& position_error_relax, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), position_error_relax(position_error_relax), body(cfg->GetBodies()[0]), helper(helper) {}

  CircleOrientationError::CircleOrientationError(LocalConfigurationPtr cfg, const Vector6d& target_pos, double orientation_error_relax, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), orientation_error_relax(orientation_error_relax), body(cfg->GetBodies()[0]), helper(helper) {}

  ExactPositionError::ExactPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), body(cfg->GetBodies()[0]), helper(helper) {}

  BallPositionError::BallPositionError(LocalConfigurationPtr cfg, const Vector6d& target_pos, double distance_error_relax, NeedleProblemHelperPtr helper) : cfg(cfg), target_pose(expUp(target_pos)), body(cfg->GetBodies()[0]), distance_error_relax(distance_error_relax), helper(helper) {}

  double cosangle(VectorXd a, VectorXd b) {
    if (a.norm() < 1e-6 || b.norm() < 1e-6) {
      return 1;
    } else {
      return a.dot(b) / (a.norm() * b.norm());
    }
  }


  VectorXd TranslationError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);

    Matrix4d current_pose = cfg->pose * expUp(a);

    Vector3d err = (current_pose.block<3, 1>(0, 3) - target_pose.block<3, 1>(0, 3)).array().abs();
    err = (err - this->position_error_relax).cwiseMax(Vector3d::Zero());

    return err;
  }


  VectorXd CircleOrientationError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);

    Matrix4d current_pose = cfg->pose * expUp(a);

    Vector3d current_rot = current_pose.topLeftCorner<3, 3>() * Vector3d(0, 0, 1);
    Vector3d target_rot = target_pose.topLeftCorner<3, 3>() * Vector3d(0, 0, 1);

    double orientation_error = fmax(1 - cosangle(current_rot, target_rot) - this->orientation_error_relax, 0);

    VectorXd err(1); err << orientation_error;
    return err;
  }


  VectorXd ExactPositionError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);
    Matrix4d current_pose = cfg->pose * expUp(a);
    return logDown(current_pose.inverse() * target_pose);
  }

  VectorXd BallPositionError::operator()(const VectorXd& a) const {
    assert(a.size() == 6);
    Matrix4d current_pose = cfg->pose * expUp(a);
    Vector3d orientation_error = logDown(current_pose.inverse() * target_pose).tail<3>();
    double distance_error = (current_pose.block<3, 1>(0, 3) - target_pose.block<3, 1>(0, 3)).norm();
    distance_error = fmax(distance_error - this->distance_error_relax, 0);

    Vector4d err;
    err(0) = distance_error;
    err.tail<3>() = orientation_error;
    return err;
  }

  PoseError::PoseError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper) : cfg0(cfg0), cfg1(cfg1), helper(helper) {}

  VectorXd PoseError::operator()(const VectorXd& a) const {
    Matrix4d pose1 = cfg0->pose * expUp(a.topRows(6));
    Matrix4d pose2 = cfg1->pose * expUp(a.middleRows(6,6));
    double Delta = a(12);
    double curvature = a(13);
    double theta = Delta * curvature;
    Vector6d v; v << 0, 0, Delta, theta, 0, 0;
    return logDown((pose1 * expUp(v)).inverse() * pose2);
  }


  SpeedDeviationError::SpeedDeviationError(double deviation, NeedleProblemHelperPtr helper) : deviation(deviation), helper(helper) {}

  VectorXd SpeedDeviationError::operator()(const VectorXd& a) const {
    Vector1d x;
    x[0] = sqrt((a.array() - deviation).square().sum()) - deviation * 0.5;
    return x;
  }

  ControlError::ControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper) : cfg0(cfg0), cfg1(cfg1), body(cfg0->GetBodies()[0]), helper(helper) {}

  VectorXd ControlError::operator()(const VectorXd& a) const {
    Matrix4d pose1 = cfg0->pose * expUp(a.topRows(6));
    Matrix4d pose2 = cfg1->pose * expUp(a.middleRows(6,6));

    double phi = a(12), Delta = a(13), curvature = a(14);
    return logDown(helper->TransformPose(pose1, phi, Delta, curvature).inverse() * pose2);

  }

  int ControlError::outputSize() const {
    return 6;
  }


  LinearizedControlError::LinearizedControlError(LocalConfigurationPtr cfg0, LocalConfigurationPtr cfg1, NeedleProblemHelperPtr helper) : cfg0(cfg0), cfg1(cfg1), body(cfg0->GetBodies()[0]), helper(helper) {}

  VectorXd LinearizedControlError::operator()(const VectorXd& a) const {
    Matrix4d pose1 = cfg0->pose * expUp(a.topRows(6));
    Matrix4d pose2 = cfg1->pose * expUp(a.middleRows(6,6));

    Vector6d x1 = a.topRows(6);
    Vector6d x2 = a.middleRows(6,6);
    double phi = a(12), Delta = a(13), curvature = a(14);
    double phi_ref = cfg0->phi, Delta_ref = cfg0->Delta, curvature_ref = cfg0->curvature;

    MatrixXd F = MatrixXd::Zero(6,6);
    F.topLeftCorner(3, 3) = -rotMat(Vector3d(Delta_ref*curvature_ref, 0, phi_ref));
    F.topRightCorner(3, 3) = -rotMat(Vector3d(0, 0, Delta_ref));
    F.bottomRightCorner(3, 3) = -rotMat(Vector3d(Delta_ref*curvature_ref, 0, phi_ref));

    MatrixXd G = MatrixXd::Zero(6,6);
    G(2, 0) = 1;
    G(3, 2) = 1;
    G(5, 1) = 1;

    MatrixXd A = F.exp();
    MatrixXd halfF = 0.5*F;
    MatrixXd B = (1/6.0) * (G + 4.0*(halfF.exp()*G) + A*G);

    Vector3d u;
    u << Delta - Delta_ref, phi - phi_ref, Delta*curvature - Delta_ref*curvature_ref;

    return x2 - A * x1 - B * u;
  }

  int LinearizedControlError::outputSize() const {
    return 6;
  }


  ChannelSurfaceDistance::ChannelSurfaceDistance(LocalConfigurationPtr cfg, NeedleProblemHelperPtr helper) : cfg(cfg), helper(helper) {} 

  VectorXd ChannelSurfaceDistance::operator()(const VectorXd& a) const {
    Matrix4d current_pose = cfg->pose * expUp(a);
    Vector3d position = current_pose.block<3, 1>(0, 3);
    double x = position.x(), y = position.y(), z = position.z();
    if (z > helper->channel_height) {
      double distance_error = x*x+y*y+(z-helper->channel_height)*(z-helper->channel_height) - (helper->channel_radius-0.2)*(helper->channel_radius-0.2);
      VectorXd ret(3); ret << distance_error, 0, 0;
      return ret;
    } else {
      double xyerror = x*x+y*y - (helper->channel_radius-0.2)*(helper->channel_radius-0.2);
      double zerror1 = z - helper->channel_height;
      double zerror2 = -0.2 - z;
      VectorXd ret(3); ret << xyerror, zerror1, zerror2;
      return ret;
    }
    
  }

  TotalCurvatureError::TotalCurvatureError(double total_curvature_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi) : total_curvature_limit(total_curvature_limit), helper(helper), pi(pi) {}

  VectorXd TotalCurvatureError::operator()(const VectorXd& a) const {
    DblVec curvatures;
    DblVec Deltas;
    curvatures = toDblVec(a.head(pi->curvature_vars.size()));
    Deltas = DblVec(pi->curvature_vars.size(), a(a.size() - 1));
    VectorXd ret(1);
    ret(0) = -this->total_curvature_limit;
    for (int i = 0; i < curvatures.size(); ++i) {
      ret(0) += curvatures[i] * Deltas[i];
    }
    return ret;
  }


  TotalRotationError::TotalRotationError(double total_rotation_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi) : total_rotation_limit(total_rotation_limit), helper(helper), pi(pi) {}

  VectorXd TotalRotationError::operator()(const VectorXd& a) const {
    DblVec phis = toDblVec(a);
    VectorXd ret(1);
    ret(0) = -this->total_rotation_limit;
    for (int i = 0; i < phis.size(); ++i) {
      ret(0) += fabs(phis[i]);
    }
    return ret;
  }


  TotalCurvatureCostError::TotalCurvatureCostError(double total_curvature_limit, NeedleProblemHelperPtr helper, NeedleProblemInstancePtr pi) : err(new TotalCurvatureError(total_curvature_limit, helper, pi)) {}

  VectorXd TotalCurvatureCostError::operator()(const VectorXd& a) const {
    VectorXd ret = err->operator()(a);
    ret(0) += err->total_curvature_limit;
    return ret;
  }

}
