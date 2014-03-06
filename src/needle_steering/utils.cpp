#include "utils.hpp"
#include <sstream>
#include <tinyxml.h>
#include <fstream>
#include <algorithm>
#include <unsupported/Eigen/MatrixFunctions>


namespace Needle {

  inline double bound_inf(double result, double bound) {
    return min(max(result, -bound), bound);
  }

  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<double>& lbs, const vector<double>& ubs, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
    int n_arr = name_prefix.size();
    assert(n_arr == newvars.size());

    vector<MatrixXi> index(n_arr);
    for (int i=0; i < n_arr; ++i) {
      newvars[i]->resize(rows, cols[i]);
      index[i].resize(rows, cols[i]);
    }

    vector<string> names;
    vector<double> all_lbs;
    vector<double> all_ubs;
    int var_idx = prob.getNumVars();
    for (int i=0; i < rows; ++i) {
      for (int k=0; k < n_arr; ++k) {
        for (int j=0; j < cols[k]; ++j) {
          index[k](i,j) = var_idx;
          names.push_back( (boost::format("%s_%i_%i")%name_prefix[k]%i%j).str() );
          all_lbs.push_back(lbs[k]);
          all_ubs.push_back(ubs[k]);
          ++var_idx;
        }
      }
    }
    prob.createVariables(names, all_lbs, all_ubs); // note that w,r, are both unbounded

    const vector<Var>& vars = prob.getVars();
    for (int k=0; k < n_arr; ++k) {
      for (int i=0; i < rows; ++i) {
        for (int j=0; j < cols[k]; ++j) {
          (*newvars[k])(i,j) = vars[index[k](i,j)];
        }
      }
    }
  }

  void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
    vector<double> lbs(newvars.size(), -INFINITY);
    vector<double> ubs(newvars.size(), INFINITY);
    AddVarArrays(prob, rows, cols, lbs, ubs, name_prefix, newvars);
  }

  void AddVarArray(OptProb& prob, int rows, int cols, double lb, double ub, const string& name_prefix, VarArray& newvars) {
    vector<VarArray*> arrs(1, &newvars);
    vector<string> prefixes(1, name_prefix);
    vector<int> colss(1, cols);
    vector<double> lbs(1, lb);
    vector<double> ubs(1, ub);
    AddVarArrays(prob, rows, colss, lbs, ubs, prefixes, arrs);
  }

  void AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars) {
    AddVarArray(prob, rows, cols, -INFINITY, INFINITY, name_prefix, newvars);
  }

  Matrix3d rotMat(const Vector3d& x) {
    Matrix3d out;
    out << 0, -x(2), x(1),
           x(2), 0, -x(0),
           -x(1), x(0), 0;
    return out;
  }

  Vector3d rotVec(const Matrix3d& X) {
    Vector3d out;
    out << X(2, 1), X(0, 2), X(1, 0);
    return out;
  }

  Matrix3d expA(const Vector3d& w) {
    double theta = w.norm();
    if (fabs(theta) < 1e-10) {
      return Matrix3d::Identity();
    } else {
      Matrix3d w_hat = rotMat(w);
      return Matrix3d::Identity() + w_hat / (theta*theta) * (1 - cos(theta)) + w_hat*w_hat / (theta*theta*theta) * (theta - sin(theta));
    }
  }

  Matrix3d logInvA(const Vector3d& w) {
    double theta = w.norm();
    Matrix3d w_hat = rotMat(w);
    if (fabs(theta) < 1e-8) {
      return Matrix3d::Identity();
    }
    return Matrix3d::Identity() - 0.5*w_hat + (2*sin(theta) - theta*(1 + cos(theta))) / (2 * theta*theta * sin(theta)) * w_hat*w_hat;
  }

  Matrix3d expRot(const Vector3d& x) {

    double rr = x.squaredNorm();
    if (fabs(rr) < 1e-10) {
      return Matrix3d::Identity();
    } else {
      double r = sqrt(rr);
      return rotMat(x * (sin(r) / r)) + Matrix3d::Identity() * cos(r) + (x*x.transpose()) * ((1 - cos(r)) / rr);
    }
    /*
    double angle;
    double rr = x.squaredNorm();
    Vector3d axis(1., 0., 0.);
    if (fabs(rr) < 1e-10) {
      angle = 0;
    }
    else
    {
      angle = sqrt(rr);
      axis = x / angle;
    }


    double cs = cos(angle);
    double sn = sin(angle);
    double oneMinusCos = ((double)1) - cs;
    double x2 = axis[0]*axis[0];
    double y2 = axis[1]*axis[1];
    double z2 = axis[2]*axis[2];
    double xym = axis[0]*axis[1]*oneMinusCos;
    double xzm = axis[0]*axis[2]*oneMinusCos;
    double yzm = axis[1]*axis[2]*oneMinusCos;
    double xSin = axis[0]*sn;
    double ySin = axis[1]*sn;
    double zSin = axis[2]*sn;

    Matrix3d res;
    res << x2*oneMinusCos + cs, xym - zSin, xzm + ySin,
    xym + zSin,
    y2*oneMinusCos + cs,
    yzm - xSin,
    xzm - ySin,
    yzm + xSin,
    z2*oneMinusCos + cs;
    return res;
    */
  }

  double ACos (double value)
  {
    if (-1.0 < value)
    {
      if (value < 1.0)
      {
        return acos(value);
      }
      else
      {
        return 0.0;
      }
    }
    else
    {
      return M_PI;
    }
  }


  double Sqrt (double value)
  {
    if (value >= 0.0)
    {
      return sqrt(value);
    }
    else
    {
      return 0.0;
    }
  }

  Vector3d logRot(const Matrix3d& X) {
    double trace = X(0, 0) + X(1, 1) + X(2, 2);
    double cs = 0.5 * (trace - 1);
    double angle = ACos(cs);  // in [0,PI]

    Vector3d axis;
    if (angle > 0.0)
    {
      if (angle < M_PI)
      {
        axis << X(2, 1) - X(1, 2),
            X(0, 2) - X(2, 0),
            X(1, 0) - X(0, 1);

        double len = axis.norm();

        if (len > 1e-8)
        {
          axis /= len;
        }
        else
        {
          axis = Vector3d::Zero();
        }

      }
      else
      {
        // angle is PI
        double halfInverse;
        if (X(0, 0) >= X(1,1))
        {
          // r00 >= r11
          if (X(0,0) >= X(2,2))
          {
            // r00 is maximum diagonal term
            axis(0) = 0.5*Sqrt(1 + X(0,0) - X(1,1) - X(2,2));
            halfInverse = 0.5/axis(0);
            axis(1) = halfInverse*X(0,1);
            axis(2) = halfInverse*X(0,2);
          }
          else
          {
            // r22 is maximum diagonal term
            axis(2) = 0.5*Sqrt(1 + X(2,2) - X(0,0) - X(1,1));
            halfInverse = 0.5/axis(2);
            axis(0) = halfInverse*X(0,2);
            axis(1) = halfInverse*X(1,2);
          }
        }
        else
        {
          // r11 > r00
          if (X(1,1) >= X(2,2))
          {
            // r11 is maximum diagonal term
            axis(1) = 0.5*Sqrt(1 + X(1,1) - X(0,0) - X(2,2));
            halfInverse  = 0.5/axis(1);
            axis(0) = halfInverse*X(0,1);
            axis(2) = halfInverse*X(1,2);
          }
          else
          {
            // r22 is maximum diagonal term
            axis(2) = 0.5*Sqrt(1 + X(2,2) - X(0,0) - X(1,1));
            halfInverse = 0.5/axis(2);
            axis(0) = halfInverse*X(0,2);
            axis(1) = halfInverse*X(1,2);
          }
        }
      }
    }
    else
    {
      // The angle is 0 and the matrix is the identity.  Any axis will
      // work, so just use the x-axis.
      axis << 1, 0, 0;
    }

    return axis * angle;
  }

  Vector3d logRot2(const Matrix3d& X) {
    // Using the old implementation since it seems more robust in practice
    Vector3d x;
    x << X(2, 1) - X(1, 2),
         X(0, 2) - X(2, 0),
         X(1, 0) - X(0, 1);
    double r = x.norm();
    double t = X(0, 0) + X(1, 1) + X(2, 2) - 1;

    if (fabs(r) < 1e-8) {
      Vector3d res = Vector3d::Zero();
      res(0) = -M_PI;
      return res;
    } else {
      return x * (atan2(r, t) / r);
    }
  }

  Matrix4d expUp(const Vector6d& x) {
    /*
    Matrix4d X = Matrix4d::Identity();
    X.block<3, 3>(0, 0) = expRot(x.tail<3>());
    X.block<3, 1>(0, 3) = expA(x.tail<3>()) * x.head<3>();
    X(3, 3) = 1;
    return X;
    */

    Matrix4d X = Matrix4d::Identity();
    X.block<3, 3>(0, 0) = rotMat(x.tail<3>());
    X.block<3, 1>(0, 3) = x.head<3>();
    X(3,3) = 0;
    return X.exp();
  }

  Vector6d logDown(const Matrix4d& X) {
    Vector6d x;
    x.tail<3>() = logRot(X.block<3, 3>(0, 0));
    x.head<3>() = (expA(x.tail<3>())).inverse() * X.block<3, 1>(0, 3);
    return x;
  }

  Matrix3d rotMatFromAxisAngle(const Vector3d& x) {
    double theta = x.norm();
    if (theta < 1e-8) {
      return Matrix3d::Identity();
    }
    Vector3d w = x / theta;
    return Matrix3d::Identity() + rotMat(w)*sin(theta) + rotMat(w)*rotMat(w)*(1-cos(theta));
  }

  Vector3d axisAngleFromRotMat(Matrix3d X) {
    Vector3d x;
    x << X(2, 1) - X(1, 2),
         X(0, 2) - X(2, 0),
         X(1, 0) - X(0, 1);
    double theta = acos((X.trace() - 1) / 2);
    if (fabs(theta) < 1e-8) {
      return Vector3d::Zero();
    } else {
      return x / (2*sin(theta)) * theta;
    }
  }

  Matrix4d se4Up(const Vector6d& x) {
    Matrix4d X = Matrix4d::Identity();
    X.block<3, 3>(0, 0) = rotMatFromAxisAngle(x.tail<3>());
    X.block<3, 1>(0, 3) = x.head<3>();
    return X;
  }

  Vector6d se4Down(const Matrix4d& X) {
    Vector6d x;
    x.tail<3>() = axisAngleFromRotMat(X.block<3, 3>(0, 0));
    x.head<3>() = X.block<3, 1>(0, 3);
    return x;
  }

  OpenRAVE::Transform matrixToTransform(const Matrix4d& X) {
    OpenRAVE::TransformMatrix M;
    M.trans.x = X(0, 3);
    M.trans.y = X(1, 3);
    M.trans.z = X(2, 3);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        M.m[row*4+col] = X(row, col);
      }
    }
    return OpenRAVE::Transform(M);
  }

  Matrix4d transformToMatrix(const OpenRAVE::Transform& M) {
    Matrix4d X = Matrix4d::Identity();
    OpenRAVE::TransformMatrix rot = OpenRAVE::geometry::matrixFromQuat(M.rot);//MatrixFrom
    X(0, 3) = M.trans.x;
    X(1, 3) = M.trans.y;
    X(2, 3) = M.trans.z;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        X(row, col) = rot.m[row*4+col];
      }
    }
    return X;
  }

  OpenRAVE::Transform vecToTransform(const Vector6d& x) {
    OpenRAVE::Transform T;
    OpenRAVE::Vector trans(x[0], x[1], x[2]);
    OpenRAVE::Vector rot(x[3], x[4], x[5]);
    T.trans = trans;
    T.rot = OpenRAVE::geometry::quatFromAxisAngle(rot);
    return T;
  }


  void saveMultiChannelBot2(const Vector3d& translation,
                            double density,
                            double cylinder_radius,
                            double cylinder_height,
                            double rotation_axis_pos, // for rotate around center: (n-1)*r, around one boundary: 0, another: 2*(n-1)*r
                            std::size_t n,
                            const string& filename)
  {
    TiXmlDocument doc;
    TiXmlElement* robot_elem = new TiXmlElement("Robot");
    robot_elem->SetAttribute("name", "channelbot");
    doc.LinkEndChild(robot_elem);

    //double delta = 0.5 * 2 * cylinder_radius * n - cylinder_radius;
    double delta = rotation_axis_pos;


    {
      TiXmlElement* kinbody_elem = new TiXmlElement("KinBody");
      robot_elem->LinkEndChild(kinbody_elem);

      TiXmlElement* body_elem = new TiXmlElement("Body");
      body_elem->SetAttribute("name", "channelink");
      body_elem->SetAttribute("type", "dynamic");
      kinbody_elem->LinkEndChild(body_elem);

      {
        TiXmlElement* translation_elem = new TiXmlElement("Translation");
        body_elem->LinkEndChild(translation_elem);
        {
          stringstream trans_v;
          trans_v << -delta << " " << 0 << " " << 0;
          TiXmlText* translation_value_elem = new TiXmlText(trans_v.str());
          translation_elem->LinkEndChild(translation_value_elem);
        }


        TiXmlElement* mass_elem = new TiXmlElement("Mass");
        mass_elem->SetAttribute("type", "mimicgeom");
        body_elem->LinkEndChild(mass_elem);
        {
          TiXmlElement* density_elem = new TiXmlElement("density");
          mass_elem->LinkEndChild(density_elem);

          stringstream density_v;
          density_v << density;
          TiXmlText* density_value_elem = new TiXmlText(density_v.str());
          density_elem->LinkEndChild(density_value_elem);
        }

        TiXmlElement* box_elem = new TiXmlElement("Geom");
        box_elem->SetAttribute("type", "box");
        body_elem->LinkEndChild(box_elem);
        {
          TiXmlElement* extents_elem = new TiXmlElement("extents");
          box_elem->LinkEndChild(extents_elem);

          stringstream extents_v;
          extents_v << 2*cylinder_radius*n / 2 << " " << 2*cylinder_radius / 2 << " " << cylinder_height / 2;
          TiXmlText* extents_value_elem = new TiXmlText(extents_v.str());
          extents_elem->LinkEndChild(extents_value_elem);


          TiXmlElement* translation_elem = new TiXmlElement("translation");
          box_elem->LinkEndChild(translation_elem);

          stringstream translation_v;
          translation_v << 0 << " " << 0 << " " << 0;
          TiXmlText* translation_value_elem = new TiXmlText(translation_v.str());
          translation_elem->LinkEndChild(translation_value_elem);


        }
      }
    }




    bool succ = doc.SaveFile(filename.c_str());
    if (!succ) cout << "save failed" << endl;
    else cout << "save succ" << endl;
    doc.Clear();
  }


  void saveMultiChannelBot(const Vector3d& translation,
                           double density,
                           double cylinder_radius,
                           double cylinder_height,
                           double rotation_axis_pos, // for rotate around center: (n-1)*r, around one boundary: 0, another: 2*(n-1)*r
                           std::size_t n,
                           const string& filename)
  {
    TiXmlDocument doc;
    TiXmlElement* robot_elem = new TiXmlElement("Robot");
    robot_elem->SetAttribute("name", "channelbot");
    doc.LinkEndChild(robot_elem);

    {
      TiXmlElement* kinbody_elem = new TiXmlElement("KinBody");
      robot_elem->LinkEndChild(kinbody_elem);


      //double delta = 0.5 * 2 * cylinder_radius * n - cylinder_radius;
      double delta = rotation_axis_pos;
      cout << "delta" << delta << endl;

      TiXmlElement* body_elem = new TiXmlElement("Body");
      body_elem->SetAttribute("name", "channelink");
      body_elem->SetAttribute("type", "dynamic");
      kinbody_elem->LinkEndChild(body_elem);

      TiXmlElement* translation_elem = new TiXmlElement("Translation");
      body_elem->LinkEndChild(translation_elem);
      {
        stringstream trans_v;
        trans_v << translation(0) << " " << translation(1) << " " << translation(2);
        TiXmlText* translation_value_elem = new TiXmlText(trans_v.str());
        translation_elem->LinkEndChild(translation_value_elem);
      }

      TiXmlElement* mass_elem = new TiXmlElement("Mass");
      mass_elem->SetAttribute("type", "mimicgeom");
      body_elem->LinkEndChild(mass_elem);
      {
        TiXmlElement* density_elem = new TiXmlElement("density");
        mass_elem->LinkEndChild(density_elem);

        stringstream density_v;
        density_v << density;
        TiXmlText* density_value_elem = new TiXmlText(density_v.str());
        density_elem->LinkEndChild(density_value_elem);
      }



      for (std::size_t i = 0; i < n; ++i)
      {
        stringstream id_v;
        id_v << i;

        TiXmlElement* cylinder_elem = new TiXmlElement("Geom");
        cylinder_elem->SetAttribute("type", "cylinder");
        body_elem->LinkEndChild(cylinder_elem);
        {
          TiXmlElement* radius_elem = new TiXmlElement("radius");
          cylinder_elem->LinkEndChild(radius_elem);

          stringstream radius_v;
          radius_v << cylinder_radius;
          TiXmlText* radius_value_elem = new TiXmlText(radius_v.str());
          radius_elem->LinkEndChild(radius_value_elem);


          TiXmlElement* height_elem = new TiXmlElement("height");
          cylinder_elem->LinkEndChild(height_elem);

          stringstream height_v;
          height_v << cylinder_height;
          TiXmlText* height_value_elem = new TiXmlText(height_v.str());
          height_elem->LinkEndChild(height_value_elem);


          TiXmlElement* translation_elem = new TiXmlElement("translation");
          cylinder_elem->LinkEndChild(translation_elem);
          {
            stringstream trans_v;
            trans_v << i * cylinder_radius * 2 - delta << " " << 0 << " " << 0;
            TiXmlText* translation_value_elem = new TiXmlText(trans_v.str());
            translation_elem->LinkEndChild(translation_value_elem);
          }


          TiXmlElement* rotationaxis_elem = new TiXmlElement("rotationaxis");
          cylinder_elem->LinkEndChild(rotationaxis_elem);

          TiXmlText* rotation_axis_value_elem = new TiXmlText("1 0 0 90");
          rotationaxis_elem->LinkEndChild(rotation_axis_value_elem);
        }


        TiXmlElement* sphere_elem = new TiXmlElement("Geom");
        sphere_elem->SetAttribute("type", "sphere");
        body_elem->LinkEndChild(sphere_elem);
        {
          TiXmlElement* translation_elem = new TiXmlElement("translation");
          sphere_elem->LinkEndChild(translation_elem);
          {
            stringstream trans_v;
            trans_v << i * cylinder_radius * 2 - delta << " " << 0 << " " << 0.5 * cylinder_height;
            TiXmlText* translation_value_elem = new TiXmlText(trans_v.str());
            translation_elem->LinkEndChild(translation_value_elem);
          }


          TiXmlElement* radius_elem = new TiXmlElement("radius");
          sphere_elem->LinkEndChild(radius_elem);

          stringstream radius_v;
          radius_v << cylinder_radius;
          TiXmlText* radius_value_elem = new TiXmlText(radius_v.str());
          radius_elem->LinkEndChild(radius_value_elem);
        }
      }
    }

    bool succ = doc.SaveFile(filename.c_str());
    if (!succ) cout << "save failed" << endl;
    else cout << "save succ" << endl;
    doc.Clear();
  }



  // in backward order
  void readInitTraj(const string& filename, vector<vector<Matrix4d> >& trajs, vector<vector<VectorXd> >& controls)
  {
    ifstream input(filename.c_str());
    if(!input)
    {
      cout << "open init traj file failed" << endl;
      return;
    }

    size_t n_trajs;
    input >> n_trajs;

    for (int k = 0; k < n_trajs; ++k)
    {
      size_t n_frames;
      input >> n_frames;

      vector<Matrix4d> poses_per_traj;
      vector<VectorXd> controls_per_traj;
      for (size_t i = 0; i < n_frames; ++i)
      {
        Matrix4d pose;
        for(size_t j = 0; j < 4; ++j)
        {
          double c1, c2, c3, c4;
          input >> c1 >> c2 >> c3 >> c4;
          pose.row(j) << c1, c2, c3, c4;
        }

        poses_per_traj.push_back(pose);

        if (i < n_frames - 1)
        {
          VectorXd control;

          while(true)
          {
            string line;
            getline(input, line);

            if(line.size() > 0)
            {
              istringstream ss(line);

              vector<double> control_data;
              double tmp;
              while (ss >> tmp)
              {
                control_data.push_back(tmp);
              }

              control = VectorXd::Map(control_data.data(), control_data.size());
              break;
            }
          }

          controls_per_traj.push_back(control);
        }

      }

      cout << poses_per_traj.size() << " " << controls_per_traj.size() << endl;


      trajs.push_back(poses_per_traj);
      controls.push_back(controls_per_traj);
    }
  }

}
