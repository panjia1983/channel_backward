#pragma once

#include "common.hpp"
#include "fwd.hpp"

namespace Needle {
  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<double>& lbs, const vector<double>& ubs, const vector<string>& name_prefix, const vector<VarArray*>& newvars);
  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars);
  void AddVarArray(OptProb& prob, int rows, int cols, double lb, double ub, const string& name_prefix, VarArray& newvars);
  void AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars);

  Matrix3d rotMat(const Vector3d& x);
  Vector3d rotVec(const Matrix3d& X);
  Matrix3d expA(const Vector3d& w);
  Matrix3d logInvA(const Vector3d& w);
  Matrix3d expRot(const Vector3d& x);
  Vector3d logRot(const Matrix3d& X);
  Matrix4d expUp(const Vector6d& x);
  Vector6d logDown(const Matrix4d& X);
  Matrix4d se4Up(const Vector6d& x);
  Vector6d se4Down(const Matrix4d& X);

  OpenRAVE::Transform matrixToTransform(const Matrix4d& X);
  OpenRAVE::Transform vecToTransform(const Vector6d& x);
  Matrix4d transformToMatrix(const OpenRAVE::Transform& M);

  void saveMultiChannelBot(const Vector3d& translation,
                           double density,
                           double cylinder_radius,
                           double cylinder_height,
                           double rotation_axis_pos,
                           std::size_t n,
                           const string& filename);

  void saveMultiChannelBot2(const Vector3d& translation,
                            double density,
                            double cylinder_radius,
                            double cylinder_height,
                            double rotation_axis_pos,
                            std::size_t n,
                            const string& filename);

  void saveExtendedObstacles(const string& original_filename,
                             const std::vector<Matrix4d>& tranfs,
                             const Vector3d& translation,
                             double density,
                             double cylinder_radius,
                             double cylinder_height,
                             double rotation_axis_pos,
                             std::size_t n,
                             const string& filename);

  void readInitTraj(const string& filename, vector<vector<Matrix4d> >& trajs, vector<vector<VectorXd> >& controls);




}
