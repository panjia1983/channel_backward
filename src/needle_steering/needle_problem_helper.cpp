#include "needle_problem_helper.hpp"
#include "utils.hpp"
#include "costs.hpp"
#include "constraints.hpp"

namespace Needle {
  template<typename T, size_t N>
  T* end(T (&ra)[N]) {
    return ra + N;
  } 

  void NeedleProblemHelper::AddRotationCost(OptProb& prob, NeedleProblemInstancePtr pi) {
    switch (rotation_cost) {
      case UseRotationQuadraticCost: {
        prob.addCost(CostPtr(new RotationQuadraticCost(pi->phivars.col(0), coeff_rotation, shared_from_this())));
        this->rotation_costs.push_back(prob.getCosts().back());
        break;
      }
      case UseRotationL1Cost: {
        prob.addCost(CostPtr(new RotationL1Cost(pi->phivars.col(0), coeff_rotation_regularization, shared_from_this())));
        this->rotation_costs.push_back(prob.getCosts().back());
        break;
      }
      SWITCH_DEFAULT;
    }
  }

  void NeedleProblemHelper::AddSpeedCost(OptProb& prob, NeedleProblemInstancePtr pi) {
    prob.addCost(CostPtr(new ConstantSpeedCost(pi->Deltavar, coeff_speed, shared_from_this(), pi)));
    this->speed_costs.push_back(prob.getCosts().back());
  }

  void NeedleProblemHelper::ConfigureProblem(OptProb& prob) {
    for (int i = 0; i < n_needles; ++i) {
      NeedleProblemInstancePtr pi(new NeedleProblemInstance());
      pi->start = starts[i];
      pi->goal = goals[i];
      pi->init_traj = init_trajs[i];
      pi->init_control = init_controls[i];
      pi->T = Ts[i];
      pi->id = i;
      pi->helper = shared_from_this();
      pi->start_position_error_relax = this->start_position_error_relax[i];
      pi->start_orientation_error_relax = this->start_orientation_error_relax[i];
      pi->goal_distance_error_relax = this->goal_distance_error_relax[i];
      CreateVariables(prob, pi);
      InitLocalConfigurations(this->robots[i], prob, pi);
      InitTrajectory(prob, pi);
      AddRotationCost(prob, pi);
      AddSpeedCost(prob, pi);
      AddStartConstraint(prob, pi);
      AddGoalConstraint(prob, pi);
      //AddFixedGoalConstraint(prob, pi);
      if (this->channel_planning) {
        AddChannelConstraint(prob, pi);
        AddTotalCurvatureConstraint(prob, pi);
        //AddTotalCurvatureCost(prob, pi);
      }

      AddTotalRotationConstraint(prob, pi);

      AddControlConstraint(prob, pi);

      AddCollisionConstraint(prob, pi);
      this->pis.push_back(pi);
    }

    for (int i = 0; i < n_needles; ++i) {
      for (int j = i+1; j < n_needles; ++j) {
        AddSelfCollisionConstraint(prob, pis[i], pis[j]);
      }
    }

    if (this->use_collision_clearance_cost) {
      AddCollisionClearanceCost(prob);
    }

    InitializeCollisionEnvironment();
  }

  void NeedleProblemHelper::AddChannelConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    for (int i = 1; i < pi->local_configs.size(); ++i) {
      VarVector vars = pi->twistvars.row(i);
      VectorOfVectorPtr f(new Needle::ChannelSurfaceDistance(pi->local_configs[i], shared_from_this()));
      VectorXd coeffs(3); coeffs << 1., 1., 1.;
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, INEQ, (boost::format("channel_surface_distance_collision_%i")%i).str())));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
    }
  }

  void NeedleProblemHelper::AddTotalCurvatureConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    VectorOfVectorPtr f(new Needle::TotalCurvatureError(this->total_curvature_limit, shared_from_this(), pi));
    VectorXd coeffs(1); coeffs << 1.;
    VarVector vars = pi->curvature_vars.flatten();
    vars = concat(vars, singleton<Var>(pi->Deltavar));
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, INEQ, (boost::format("total_curvature_constraint_collision_%i")%pi->id).str())));
  }

  void NeedleProblemHelper::AddTotalRotationConstraint(OptProb& prob, NeedleProblemInstancePtr pi)
  {
    /*
    VectorOfVectorPtr f(new Needle::TotalCurvatureError(this->total_rotation_limit, shared_from_this(), pi));
    VectorXd coeffs(1); coeffs << 1.;
    VarVector vars = pi->phivars.flatten();
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, INEQ, (boost::format("total_rotation_constraint_collision_%i")%pi->id).str())));
    */

    VarVector vars = pi->phivars.flatten();
    AffExpr total_rotation;
    for (int i = 0; i < vars.size(); ++i)
      exprInc(total_rotation, vars[i]);
    exprSub(total_rotation, -this->total_rotation_limit);
    prob.addLinearConstraint(total_rotation, EQ);
  }

  void NeedleProblemHelper::AddTotalCurvatureCost(OptProb& prob, NeedleProblemInstancePtr pi) {
    VectorOfVectorPtr f(new Needle::TotalCurvatureCostError(this->total_curvature_limit, shared_from_this(), pi));
    VectorXd coeffs(1); coeffs << 2.;
    VarVector vars = pi->curvature_vars.flatten();
    vars = concat(vars, singleton<Var>(pi->Deltavar));
    prob.addCost(CostPtr(new CostFromErrFunc(f, vars, coeffs, ABS, (boost::format("total_curvature_cost_%i")%pi->id).str())));
  }




  void NeedleProblemHelper::InitOptimizeVariables(OptimizerT& opt) {
    DblVec initVec;
    for (int k = 0; k < n_needles; ++k) {
      NeedleProblemInstancePtr pi = pis[k];
      if (pi->initVec.size() == 0) {

        // Initialize twistvars
        for (int i = 0; i <= pi->T; ++i) {
          for (int j = 0; j < n_dof; ++j) {
            pi->initVec.push_back(0.);
          }
        }

        if (this->use_init_traj)
        {
          // Initialize phivars
          for (int i = 0; i < pi->T; ++i)
            pi->initVec.push_back(pi->init_control[i](2));

          // Initialize Delta
          pi->initVec.push_back(pi->init_control[0](0));

          // Initialize time frame curvature
          for (int i = 0; i < pi->T; ++i)
            pi->initVec.push_back(pi->init_control[0](1));
        }
        else
        {
          // Initialize phivars
          for (int i = 0; i < pi->T; ++i) {
            pi->initVec.push_back(0.);
          }

          // Initialize Delta
          pi->initVec.push_back(pi->Delta_lb);

          // Initialize time frame curvature
          for (int i = 0; i < pi->T; ++i) {
            pi->initVec.push_back(1.0 / (r_min*20));
          }
        }

        for (int i = 0; i < pi->initVec.size(); ++i) {
          initVec.push_back(pi->initVec[i]);
        }
      }
    }
    opt.initialize(initVec);
  }

  Matrix4d NeedleProblemHelper::TransformPose(const Matrix4d& pose, double phi, double Delta, double curvature) const {
    double theta;
    theta = Delta * curvature;
    Vector6d w; w << 0, 0, Delta, theta, 0, phi;
    return pose * expUp(w);
  }

  double NeedleProblemHelper::GetPhi(const DblVec& x, int i, NeedleProblemInstancePtr pi) const {
    return x[pi->phivars.row(i)[0].var_rep->index];
  }

  double NeedleProblemHelper::GetDelta(const DblVec& x, int i, NeedleProblemInstancePtr pi) const {
    return x[pi->Deltavar.var_rep->index];
  }
  
  double NeedleProblemHelper::GetCurvature(const DblVec& x, int i, NeedleProblemInstancePtr pi) const {
    return x[pi->curvature_vars.row(i)[0].var_rep->index];
  }

  bool NeedleProblemHelper::OptimizerCallback(OptProb*, DblVec& x) {
    
    switch (method) {
      case Colocation: {
        for (int i = 0; i < n_needles; ++i) {
          MatrixXd twistvals = getTraj(x, pis[i]->twistvars);
          for (int j = 0; j < pis[i]->local_configs.size(); ++j) {
            pis[i]->local_configs[j]->pose = pis[i]->local_configs[j]->pose * expUp(twistvals.row(j));
          }
          setVec(x, pis[i]->twistvars.m_data, DblVec(pis[i]->twistvars.size(), 0));
        }
        return false;
      }
      case Shooting: {
        // execute the control input to set local configuration poses
        for (int i = 0; i < n_needles; ++i) {
          MatrixXd twistvals = getTraj(x, pis[i]->twistvars);

          //pis[i]->local_configs[pis[i]->T]->pose = expUp(pis[i]->goal);//*= expUp(twistvals.row(pis[i]->T));
          cout << pis[i]->local_configs[0]->pose << endl;
          pis[i]->local_configs[0]->pose = expUp(pis[i]->goal);//*= expUp(twistvals.row(pis[i]->T));
          cout << pis[i]->goal << endl;
          cout << pis[i]->local_configs[0]->pose << endl;

          //for (int j = pis[i]->T-1; j >=0; --j) {
          for (int j = 1; j <= pis[i]->T; ++j) {
            double phi = GetPhi(x, j-1, pis[i]);
            double Delta = GetDelta(x, j-1, pis[i]);
            double curvature = GetCurvature(x, j-1, pis[i]);
            cout << Delta << " " << phi << " " << curvature << endl;
            pis[i]->local_configs[j]->pose = TransformPose(pis[i]->local_configs[j-1]->pose, phi, Delta, curvature);
          }
          setVec(x, pis[i]->twistvars.m_data, DblVec(pis[i]->twistvars.size(), 0));
        }
        return true; // should_recompute = true: needs to recompute cnts and costs
      }
      SWITCH_DEFAULT;
    }
  }

  void NeedleProblemHelper::ConfigureOptimizer(OptimizerT& opt) {
    opt.max_iter_ = 100;    
    opt.improve_ratio_threshold_ = this->improve_ratio_threshold;
    opt.trust_shrink_ratio_ = this->trust_shrink_ratio;
    opt.trust_expand_ratio_ = this->trust_expand_ratio;
    opt.trust_box_size_ = this->trust_box_size;
    opt.record_trust_region_history_ = this->record_trust_region_history;
    opt.max_merit_coeff_increases_ = this->max_merit_coeff_increases;
    opt.merit_error_coeff_ = this->merit_error_coeff;

    InitOptimizeVariables(opt);

    opt.addCallback(boost::bind(&Needle::NeedleProblemHelper::OptimizerCallback, this, _1, _2));
  }

  void NeedleProblemHelper::CreateVariables(OptProb& prob, NeedleProblemInstancePtr pi) {
    AddVarArray(prob, pi->T+1, n_dof, "twist", pi->twistvars);
    AddVarArray(prob, pi->T, 1, -PI, PI, "phi", pi->phivars);
    pi->Delta_lb = (expUp(pi->goal).topRightCorner<3, 1>() - expUp(pi->start).topRightCorner<3, 1>()).norm() / pi->T;
    pi->Deltavar = prob.createVariables(singleton<string>("Delta"), singleton<double>(pi->Delta_lb),singleton<double>(INFINITY))[0];
    AddVarArray(prob, pi->T, 1, 1. / (r_min*30), 1. / r_min, "curvature", pi->curvature_vars);
  }

  void NeedleProblemHelper::InitLocalConfigurations(const KinBodyPtr robot, OptProb& prob, NeedleProblemInstancePtr pi) {
    for (int i = 0; i <= pi->T; ++i) {
      pi->local_configs.push_back(LocalConfigurationPtr(new LocalConfiguration(robot)));
    }
  }

  void NeedleProblemHelper::InitTrajectory(OptProb& prob, NeedleProblemInstancePtr pi) {
    MatrixXd initTraj(pi->T+1, n_dof);

    if (this->use_init_traj)
    {
      for (int i = 0; i <= pi->T; ++i)
        initTraj.row(i) = pi->init_traj[i];
    }
    else
    {
      for (int idof = 0; idof < n_dof; ++idof) {
        initTraj.col(idof) = VectorXd::LinSpaced(pi->T+1, pi->goal[idof], pi->start[idof]);
      }
    }

    for (int i = 0; i <= pi->T; ++i) {
      pi->local_configs[i]->pose = expUp(initTraj.row(i));
    }
  }

  void NeedleProblemHelper::AddStartConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    VarVector vars = pi->twistvars.row(pi->T);

    if (pi->start_position_error_relax.norm() > 1e-4 || pi->start_orientation_error_relax > 1e-4)
    {
      VectorOfVectorPtr f_orientation;
      VectorOfVectorPtr f_translation;
      VectorXd coeffs_orientation;
      VectorXd coeffs_translation;

      if (this->channel_planning)
      {

        Vector6d start_cons; start_cons << 0, 0, 0, 0, -3.14, 0;
        start_cons = logDown(se4Up(start_cons));
        f_orientation.reset(new Needle::CircleOrientationError(pi->local_configs[pi->T], start_cons, pi->start_orientation_error_relax, shared_from_this()));
        f_translation.reset(new Needle::TranslationError(pi->local_configs[pi->T], start_cons, pi->start_position_error_relax, shared_from_this()));
        coeffs_orientation = VectorXd(1);
        coeffs_translation = VectorXd(3);
      }
      else
      {
        f_orientation.reset(new Needle::CircleOrientationError(pi->local_configs[pi->T], pi->start, pi->start_orientation_error_relax, shared_from_this()));
        f_translation.reset(new Needle::TranslationError(pi->local_configs[pi->T], pi->start, pi->start_position_error_relax, shared_from_this()));
        coeffs_orientation = VectorXd(1);
        coeffs_translation = VectorXd(3);
      }

      coeffs_orientation << 1;
      coeffs_translation << 1, 1, 1;

      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f_orientation, vars, coeffs_orientation, EQ, "entry")));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f_translation, vars, coeffs_translation, EQ, "entry")));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());

    }
    else
    {
      VectorOfVectorPtr f(new Needle::ExactPositionError(pi->local_configs[pi->T], pi->start, shared_from_this()));
      Vector6d coeffs; coeffs << 1., 1., 1., 1., 1., 1.;
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, "entry")));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
    }

  }

  void NeedleProblemHelper::AddGoalConstraint(OptProb& prob, NeedleProblemInstancePtr pi)
  {
    VarVector vars = pi->twistvars.row(0);
    for (int i = 0; i < vars.size(); ++i)
    {
      prob.addLinearConstraint(AffExpr(vars[i]), EQ);
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
    }

  }

  void NeedleProblemHelper::AddControlConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    for (int i = 0; i < pi->T; ++i) {
      VarVector vars = concat(concat(pi->twistvars.row(i), pi->twistvars.row(i+1)), pi->phivars.row(i));
      vars.push_back(pi->Deltavar);
      vars = concat(vars, pi->curvature_vars.row(i));
      VectorOfVectorPtr f(new Needle::ControlError(pi->local_configs[i], pi->local_configs[i+1], shared_from_this()));
      VectorXd coeffs = VectorXd::Ones(boost::static_pointer_cast<Needle::ControlError>(f)->outputSize());
      coeffs.tail<3>() *= this->coeff_orientation_error;
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("control%i")%i).str())));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
    }
  }

  void NeedleProblemHelper::AddPoseConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    for (int i = 0; i < pi->T; ++i) {
      VarVector vars = concat(pi->twistvars.row(i), pi->twistvars.row(i+1));
      vars.push_back(pi->Deltavar);
      vars = concat(vars, pi->curvature_vars.row(i));

      VectorOfVectorPtr f(new Needle::PoseError(pi->local_configs[i], pi->local_configs[i+1], shared_from_this()));
      VectorXd coeffs = VectorXd::Ones(6);
      coeffs(3) = coeffs(4) = 0;
      coeffs.tail<3>() *= this->coeff_orientation_error;
      prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("control%i")%i).str())));
      pi->dynamics_constraints.push_back(prob.getConstraints().back());
    }
  }

  void NeedleProblemHelper::AddCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr pi) {
    if (continuous_collision) {
      for (int i = 0; i < pi->T; ++i) {
        prob.addConstraint(ConstraintPtr(new CollisionConstraint(collision_dist_pen, collision_coeff, pi->local_configs[i], pi->local_configs[i+1], pi->twistvars.row(i), pi->twistvars.row(i+1))));
        pi->collision_constraints.push_back(prob.getConstraints().back());
        prob.getConstraints().back()->setName((boost::format("collision_obstacle_%i (needle %i)")%i%pi->id).str());
      }
    } else {
      for (int i = 0; i <= pi->T; ++i) {
        prob.addConstraint(ConstraintPtr(new CollisionConstraint(collision_dist_pen, collision_coeff, pi->local_configs[i], pi->twistvars.row(i))));
        pi->collision_constraints.push_back(prob.getConstraints().back());
        prob.getConstraints().back()->setName((boost::format("collision_obstacle_%i (needle %i)")%i%pi->id).str());
      }
    }
  }

  void NeedleProblemHelper::AddSelfCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr piA, NeedleProblemInstancePtr piB) {
    if (continuous_collision) {
      for (int i = 0; i < piA->T; ++i) {
        for (int j = 0; j < piB->T; ++j) {
          prob.addConstraint(ConstraintPtr(new CollisionConstraint(collision_dist_pen, collision_coeff, piA->local_configs[i], piA->local_configs[i+1], piB->local_configs[j], piB->local_configs[j+1], piA->twistvars.row(i), piA->twistvars.row(i+1), piB->twistvars.row(j), piB->twistvars.row(j+1))));
          self_collision_constraints.push_back(prob.getConstraints().back());
          prob.getConstraints().back()->setName((boost::format("self_collision_%i_%i")%i%j).str());
        }
      }
    } else {
      throw std::runtime_error("Not implemented");
    }
  }

  void NeedleProblemHelper::AddCollisionClearanceCost(OptProb& prob) {
    prob.addCost(CostPtr(new NeedleCollisionClearanceCost(shared_from_this(), this->collision_clearance_coeff))); 
    this->clearance_costs.push_back(prob.getCosts().back());
  }

  void NeedleProblemHelper::InitializeCollisionEnvironment() {
    EnvironmentBasePtr env = pis[0]->local_configs[0]->GetEnv();
    vector<KinBodyPtr> bodies; env->GetBodies(bodies);
    double contact_distance = collision_dist_pen + 0.05;
    if (this->use_collision_clearance_cost) {
      contact_distance = fmax(contact_distance, collision_clearance_threshold);
    }
    CollisionChecker::GetOrCreate(*env)->SetContactDistance(contact_distance);


    for (int k = 0; k < n_needles; ++k) {
      for (int i=0; i < bodies.size(); ++i) {
        if (std::find(ignored_kinbody_names.begin(), ignored_kinbody_names.end(), bodies[i]->GetName()) != ignored_kinbody_names.end()) {
          BOOST_FOREACH(const KinBody::LinkPtr& robot_link, robots[k]->GetLinks()) {
            BOOST_FOREACH(const KinBody::LinkPtr& body_link, bodies[i]->GetLinks()) {
              CollisionChecker::GetOrCreate(*env)->ExcludeCollisionPair(*body_link, *robot_link);//*bodies[i]->GetLinks()[0], *robots[k]->GetLinks()[0]);
            }
          }
        }
      }
    }
  }

  void NeedleProblemHelper::InitParameters() {
    this->r_min = 5;
    this->n_dof = 6;

    this->method = NeedleProblemHelper::Shooting;
    this->rotation_cost = NeedleProblemHelper::UseRotationQuadraticCost;
    this->continuous_collision = true;
    this->goal_orientation_constraint = false;
    this->use_collision_clearance_cost = true;

    // parameters for the optimizer
    this->improve_ratio_threshold = 0.2;
    this->trust_shrink_ratio = 0.9;
    this->trust_expand_ratio = 1.2;
    this->trust_box_size = .1;
    this->record_trust_region_history = false;
    this->merit_error_coeff = 5;
    this->max_merit_coeff_increases = 5;

    this->coeff_rotation = 1.;
    this->coeff_speed = 1.;
    this->coeff_rotation_regularization = 0.1;
    this->coeff_orientation_error = 1;
    this->collision_dist_pen = 0.05;
    this->collision_coeff = 10;
    this->collision_clearance_coeff = 1;
    this->collision_clearance_threshold = 1;
    this->total_curvature_limit = 1.57;
    this->total_rotation_limit = 3.14;

    this->channel_radius = 2.5;
    this->channel_height = 7;
    this->channel_safety_margin = 0.25;
    this->channel_planning = true;

    const char *ignored_kinbody_c_strs[] = { "KinBodyProstate", "KinBodyDermis", "KinBodyEpidermis", "KinBodyHypodermis", "KinBodyEntryRegion", "ImplantKinBody" };
    this->ignored_kinbody_names = vector<string>(ignored_kinbody_c_strs, end(ignored_kinbody_c_strs));
  }

  void NeedleProblemHelper::InitParametersFromConsole(int argc, char** argv) {
    Clear();
    InitParameters();
    
    Config config;
    config.add(new Parameter<int>("method", &this->method, "method"));
    config.add(new Parameter<int>("rotation_cost", &this->rotation_cost, "rotation_cost"));
    config.add(new Parameter<double>("coeff_rotation_regularization", &this->coeff_rotation_regularization, "coeff_rotation_regularization"));
    config.add(new Parameter<double>("coeff_rotation", &this->coeff_rotation, "coeff_rotation"));
    config.add(new Parameter<double>("coeff_speed", &this->coeff_speed, "coeff_speed"));
    config.add(new Parameter<double>("coeff_orientation_error", &this->coeff_orientation_error, "coeff_orientation_error"));
    config.add(new Parameter<double>("r_min", &this->r_min, "r_min"));
    config.add(new Parameter<double>("improve_ratio_threshold", &this->improve_ratio_threshold, "improve_ratio_threshold"));
    config.add(new Parameter<double>("trust_shrink_ratio", &this->trust_shrink_ratio, "trust_shrink_ratio"));
    config.add(new Parameter<double>("trust_expand_ratio", &this->trust_expand_ratio, "trust_expand_ratio"));
    config.add(new Parameter<double>("collision_dist_pen", &this->collision_dist_pen, "collision_dist_pen"));
    config.add(new Parameter<double>("collision_coeff", &this->collision_coeff, "collision_coeff"));
    config.add(new Parameter<double>("collision_clearance_coeff", &this->collision_clearance_coeff, "collision_clearance_coeff"));
    config.add(new Parameter<double>("collision_clearance_threshold", &this->collision_clearance_threshold, "collision_clearance_threshold"));
    config.add(new Parameter<double>("merit_error_coeff", &this->merit_error_coeff, "merit_error_coeff"));
    config.add(new Parameter<double>("total_curvature_limit", &this->total_curvature_limit, "total_curvature_limit"));
    config.add(new Parameter<double>("total_rotation_limit", &this->total_rotation_limit, "total_rotation_limit"));
    config.add(new Parameter<bool>("use_collision_clearance_cost", &this->use_collision_clearance_cost, "use_collision_clearance_cost"));
    config.add(new Parameter<bool>("record_trust_region_history", &this->record_trust_region_history, "record_trust_region_history"));
    config.add(new Parameter<bool>("continuous_collision", &this->continuous_collision, "continuous_collision"));
    config.add(new Parameter<bool>("goal_orientation_constraint", &this->goal_orientation_constraint, "goal_orientation_constraint"));
    config.add(new Parameter<bool>("channel_planning", &this->channel_planning, "channel_planning"));

    CommandParser parser(config);
    parser.read(argc, argv, true);

  }

  void NeedleProblemHelper::Clear() {
    starts.clear();
    goals.clear();
    if (robots.size() > 0) {
      EnvironmentBasePtr env = pis[0]->local_configs[0]->GetEnv();
      for (int i = 0; i < n_needles; ++i) {
        env->Remove(robots[i]);
        trajopt::RemoveUserData(*robots[i], "bt");
      }
    }
    robots.clear();
    self_collision_constraints.clear();
    start_position_error_relax.clear();
    start_orientation_error_relax.clear();
    goal_distance_error_relax.clear();
    pis.clear();
    Ts.clear();
    n_needles = 0;
    rotation_costs.clear();
    speed_costs.clear();
    clearance_costs.clear();
  }

  void NeedleProblemHelper::AddNeedlesToBullet(OptimizerT& opt) {
    for (int i = 0; i < n_needles; ++i) {
      AddNeedleToBullet(pis[i], opt);
    }
  }

  void NeedleProblemHelper::AddNeedleToBullet(NeedleProblemInstancePtr pi, OptimizerT& opt) {
    
    EnvironmentBasePtr env = pi->local_configs[0]->GetEnv();
    DblVec x = opt.x();
    MatrixXd twistvals = getTraj(x, pi->twistvars);
    boost::shared_ptr<BulletCollisionChecker> cc = boost::dynamic_pointer_cast<BulletCollisionChecker>(CollisionChecker::GetOrCreate(*env));
    for (int i = 0; i < pi->T; ++i) {
      vector<KinBody::LinkPtr> links;
      vector<int> inds;
      pi->local_configs[i]->GetAffectedLinks(links, true, inds);
      cc->AddCastHullShape(*pi->local_configs[i], *pi->local_configs[i+1], links, toDblVec(twistvals.row(i)), toDblVec(twistvals.row(i+1)));
    }
  }

  vector<VectorXd> NeedleProblemHelper::GetSolutions(OptimizerT& opt) {
    vector<VectorXd> ret;
    for (int i = 0; i < n_needles; ++i) {
      ret.push_back(pis[i]->GetSolution(opt));
    }
    return ret;
  }

  vector< vector<Vector6d> > NeedleProblemHelper::GetStates(OptimizerT& opt) {
    vector< vector<Vector6d> > ret;
    for (int i = 0; i < n_needles; ++i) {
      ret.push_back(pis[i]->GetStates(opt));
    }
    return ret;
  }

  void NeedleProblemHelper::SetSolutions(const vector<VectorXd>& sol, OptimizerT& opt) {
    assert (sol.size() == n_needles);
    for (int i = 0; i < n_needles; ++i) {
      pis[i]->SetSolution(sol[i], opt);
    }
  }

  vector<VectorXd> NeedleProblemHelper::GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol) {
    vector<VectorXd> ret;
    assert (sol.size() == n_needles);
    if (this->Ts.front() > 1) {
      ret.push_back(pis.front()->GetSolutionWithoutFirstTimestep(sol.front()));
    }

    for (int i = 1; i < n_needles; ++i) {
      ret.push_back(sol[i]);
    }
    return ret;
  }

  void NeedleProblemHelper::IntegrateControls(DblVec& x) {
    for (int i = 0; i < n_needles; ++i) {
      if (i == 0) {
        pis[i]->local_configs[0]->pose = expUp(pis[i]->goal);
      }
      for (int j = 1; j <= pis[i]->T; ++j) {
        double phi = GetPhi(x, j-1, pis[i]);
        double Delta = GetDelta(x, j-1, pis[i]);
        double curvature = GetCurvature(x, j-1, pis[i]);
        pis[i]->local_configs[j]->pose = TransformPose(pis[i]->local_configs[j-1]->pose, phi, Delta, curvature);
      }
      setVec(x, pis[i]->twistvars.m_data, DblVec(pis[i]->twistvars.size(), 0));
    }
  }

  vector<DblVec> NeedleProblemHelper::GetPhis(OptimizerT& opt) {
    vector<DblVec> phiss;
    DblVec& x = opt.x();
    for (int i = 0; i < this->pis.size(); ++i) {
      DblVec phis;
      for (int j = 0; j < pis[i]->T; ++j) {
        phis.push_back(GetPhi(x, j, pis[i]));
      }
      phiss.push_back(phis);
    }
    return phiss;
  }

  vector<DblVec> NeedleProblemHelper::GetDeltas(OptimizerT& opt) {
    vector<DblVec> Deltass;
    DblVec& x = opt.x();
    for (int i = 0; i < this->pis.size(); ++i) {
      DblVec Deltas;
      for (int j = 0; j < pis[i]->T; ++j) {
        Deltas.push_back(GetDelta(x, j, pis[i]));
      }
      Deltass.push_back(Deltas);
    }
    return Deltass;
  }

  vector<DblVec> NeedleProblemHelper::GetCurvatures(OptimizerT& opt) {
    vector<DblVec> curvaturess;
    DblVec& x = opt.x();
    for (int i = 0; i < this->pis.size(); ++i) {
      DblVec curvatures;
      for (int j = 0; j < pis[i]->T; ++j) {
        curvatures.push_back(GetCurvature(x, j, pis[i]));
      }
      curvaturess.push_back(curvatures);
    }
    return curvaturess;
  }

}
