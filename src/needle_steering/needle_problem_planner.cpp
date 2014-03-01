#include "needle_problem_planner.hpp"
#include "utils.hpp"
#include "traj_plotter.hpp"
#include "needle_collision_hash.hpp"

namespace Needle {

  inline double rndnum() {
	  return ((double) random()) / RAND_MAX;
  }

  inline double normal() {
    double u_1 = 0;
    while (u_1 == 0) {
      u_1 = rndnum();
    }
    double u_2 = 0;
    while (u_2 == 0) {
      u_2 = rndnum();
    }
    return sqrt(-2*log(u_1)) * sin(2*PI*u_2);
  }

  NeedleProblemPlanner::NeedleProblemPlanner(int argc, char **argv) :
    argc(argc),
    argv(argv),
    helper(new NeedleProblemHelper()),
    stage_plotting(false),
    stage_result_plotting(false),
    verbose(false),
    env_transparency(0.1),
    is_first_needle_run(true),
    deviation(INFINITY),
    separate_planning_first(true),
    max_sequential_solves(10),
    current_sim_index(0),
    current_open_sim_index(0),
    channel_planning(false),
    current_converged(false),
    perturb_initialization(false),
    seq_result_plotting(false),
    use_colocation_correction(false),
    noise_scale(1.),
    data_dir(get_current_directory() + "/../data") {

    vector<string> entry_string_vec;
    vector<string> final_string_vec;

    vector<double> entry_position_error_relax_x;
    vector<double> entry_position_error_relax_y;
    vector<double> entry_position_error_relax_z;

    int T = 25;
    n_channels = 1;
    use_init_traj = false;
    
    Config config;
    config.add(new Parameter<bool>("stage_plotting", &this->stage_plotting, "stage_plotting"));
    config.add(new Parameter<bool>("seq_result_plotting", &this->seq_result_plotting, "seq_result_plotting"));
    config.add(new Parameter<bool>("stage_result_plotting", &this->stage_result_plotting, "stage_result_plotting"));
    config.add(new Parameter<bool>("verbose", &this->verbose, "verbose"));
    config.add(new Parameter<bool>("separate_planning_first", &this->separate_planning_first, "separate_planning_first"));
    config.add(new Parameter<bool>("simultaneous_planning", &this->simultaneous_planning, "simultaneous_planning"));
    config.add(new Parameter<bool>("use_colocation_correction", &this->use_colocation_correction, "use_colocation_correction"));
    config.add(new Parameter<double>("env_transparency", &this->env_transparency, "env_transparency"));
    config.add(new Parameter<double>("noise_scale", &this->noise_scale, "noise_scale"));
    config.add(new Parameter<string>("data_dir", &this->data_dir, "data_dir"));
    config.add(new Parameter<string>("env_file_path", &this->env_file_path, "env_file_path"));
    config.add(new Parameter<string>("robot_file_path", &this->robot_file_path, "robot_file_path"));
    config.add(new Parameter<string>("init_traj_path", &this->init_traj_path, "init_traj_path"));
    config.add(new Parameter<bool>("use_init_traj", &this->use_init_traj, "use_init_traj"));
    config.add(new Parameter< vector<string> >("entry_vec", &entry_string_vec, "s"));
    config.add(new Parameter< vector<string> >("final_vec", &final_string_vec, "g"));
    config.add(new Parameter< vector<double> >("entry_position_error_relax_x", &entry_position_error_relax_x, "entry_position_error_relax_x"));
    config.add(new Parameter< vector<double> >("entry_position_error_relax_y", &entry_position_error_relax_y, "entry_position_error_relax_y"));
    config.add(new Parameter< vector<double> >("entry_position_error_relax_z", &entry_position_error_relax_z, "entry_position_error_relax_z"));
    config.add(new Parameter< vector<double> >("entry_orientation_error_relax", &this->entry_orientation_error_relax, "entry_orientation_error_relax"));
    config.add(new Parameter< vector<double> >("final_distance_error_relax", &this->final_distance_error_relax, "final_distance_error_relax"));
    config.add(new Parameter<bool>("channel_planning", &this->channel_planning, "channel_planning"));
    config.add(new Parameter<int>("n_channels", &this->n_channels, "n_channels"));
    config.add(new Parameter<bool>("perturb_initialization", &this->perturb_initialization, "perturb_initialization"));
    config.add(new Parameter<int>("T", &T, "T"));
    config.add(new Parameter<int>("max_sequential_solves", &max_sequential_solves, "max_sequential_solves"));
    CommandParser parser(config);
    parser.read(argc, argv, true);

    if (channel_planning) {
      if (entry_string_vec.size() == 0) {

        entry_string_vec.push_back("-1.25,0.0,0,0,0,0");
        final_string_vec.push_back("-1.25,0.0,7,-1.1780972450961724,-0.0,-0.0");
        entry_string_vec.push_back("1.25,0.0,0,0,0,0");
        final_string_vec.push_back("1.25,0.0,7,1.1780972450961724,0.0,0.0");
        entry_string_vec.push_back("0.0,-1.25,0,0,0,0");
        final_string_vec.push_back("0.0,-1.25,7,-0.0,-1.1780972450961724,-0.0");
        entry_string_vec.push_back("0.0,1.25,0,0,0,0");
        final_string_vec.push_back("0.0,1.25,7,0.0,1.1780972450961724,0.0");
        entry_string_vec.push_back("-1.5556349186104046,-1.5556349186104048,0,0,0,0");
        final_string_vec.push_back("-1.5556349186104046,-1.5556349186104048,3.5,0.0,0.0,0.0");
        entry_string_vec.push_back("-2.12503681783595,-0.5694018992255463,0,0,0,0");
        final_string_vec.push_back("-2.12503681783595,-0.5694018992255463,3.5,0.0,0.0,0.0");
        entry_string_vec.push_back("-2.1250368178359507,0.5694018992255448,0,0,0,0");
        final_string_vec.push_back("-2.1250368178359507,0.5694018992255448,3.5,0.0,0.0,0.0");
        entry_string_vec.push_back("-1.555634918610405,1.5556349186104046,0,0,0,0");
        final_string_vec.push_back("-1.555634918610405,1.5556349186104046,3.5,0.0,0.0,0.0");
        entry_string_vec.push_back("1.4142135623730951,1.414213562373095,0,0,0,0");
        final_string_vec.push_back("1.4142135623730951,1.414213562373095,3.5,0.0,0.39269908169872414,0.0");
        entry_string_vec.push_back("2.0,0.0,0,0,0,0");
        final_string_vec.push_back("2.0,0.0,3.5,0.0,0.39269908169872414,0.0");
        entry_string_vec.push_back("1.4142135623730951,-1.414213562373095,0,0,0,0");
        final_string_vec.push_back("1.4142135623730951,-1.414213562373095,3.5,0.0,0.39269908169872414,0.0");
      }


      for (int i = 0; i < final_string_vec.size(); ++i) {
        this->entry_position_error_relax.push_back(Vector3d(2.5, 2.5, 0.1));
        this->entry_orientation_error_relax.push_back(0.5); // 0.01, 0.1744
        this->final_distance_error_relax.push_back(0);
      }


      if (this->env_file_path.length() == 0) {
        this->env_file_path = data_dir + "/channel.env.xml";
      }
      if (this->robot_file_path.length() == 0) {
        this->robot_file_path = data_dir + "/channelbot.xml";
      }
    } else {
      if (entry_string_vec.size() == 0) {
        entry_string_vec.push_back("-7.5,5.75,0,0,1.57,0");
        final_string_vec.push_back("-3.2396,6.46645,0.301649,0,1.57,0");
        entry_string_vec.push_back("-7.5,4.75,0,0,1.57,0");
        final_string_vec.push_back("-2.71912,8.00334,-1.12736,0,1.57,0");
        entry_string_vec.push_back("-7.5,5.25,0,0,1.57,0");
        final_string_vec.push_back("-1.99682,7.43527,-1.85617,0,1.57,0");
        entry_string_vec.push_back("-7.5,5.35,0,0,1.57,0");
        final_string_vec.push_back("-2.0386,7.0732,0.493712,0,1.57,0");
        entry_string_vec.push_back("-7.5,5.45,0,0,1.57,0");
        final_string_vec.push_back("-2.74817,5.83943,0.104912,0,1.57,0");

        for (int i = 0; i < final_string_vec.size(); ++i) {
          this->entry_position_error_relax.push_back(Vector3d(0.05, 2.5, 1.25));
          this->entry_orientation_error_relax.push_back(0.08);
          this->final_distance_error_relax.push_back(0.125);
        }
      }
      if (this->env_file_path.length() == 0) {
        this->env_file_path = data_dir + "/prostate.env.xml";
      }
      if (this->robot_file_path.length() == 0) {
        this->robot_file_path = data_dir + "/needlebot.xml";
      }
    }

    if (entry_string_vec.size() != final_string_vec.size()) {
      throw std::runtime_error("The number of entry and final vectors must be the same!");
    }

    if (entry_string_vec.size() == 0) {
      throw std::runtime_error("You must provide at least 1 entry and 1 final vector.");
    }

    RaveInitialize(false, verbose ? Level_Debug : Level_Fatal);
    this->env = RaveCreateEnvironment();
    this->env->StopSimulation();

    OSGViewerPtr viewer;
    if (this->stage_plotting || this->stage_result_plotting ) {
      viewer = OSGViewer::GetOrCreate(env);
      assert(viewer);
    }

    this->env->Load(this->env_file_path);

    if (this->stage_plotting || this->stage_result_plotting ) {
      viewer->SetAllTransparency(this->env_transparency);
    }


    if (channel_planning)
    {
      Vector3d t;
      t << 0, 0, 0;
      saveMultiChannelBot(t, 1, 0.1, 0.5, this->n_channels, this->data_dir + "/test.xml");
      this->robot_file_path = this->data_dir + "/test.xml";
    }

    KinBodyPtr robot = this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path);

    if (this->init_traj_path == "")
    {
      this->n_needles = entry_string_vec.size();
      this->entries.clear();
      this->finals.clear();
      this->init_trajs.clear();
      this->init_controls.clear();

      this->sim_in_collision = vector<bool>(this->n_needles, false);
      this->distance_to_final = vector<double>(this->n_needles, 0);

      for (int i = 0; i < n_needles; ++i) {
        DblVec entry_vec;
        DblVec final_vec;

        strtk::parse(entry_string_vec[i], ",", entry_vec);
        strtk::parse(final_string_vec[i], ",", final_vec);

        entries.push_back(logDown(se4Up(toVectorXd(entry_vec))));
        finals.push_back(logDown(se4Up(toVectorXd(final_vec))));
      }

      init_trajs.resize(n_needles);
      init_controls.resize(n_needles);
    }
    else
    {
      vector<Matrix4d> poses;
      vector<VectorXd> controls;
      readInitTraj(this->init_traj_path, poses, controls);

      for (size_t i = 0; i < poses.size(); ++i)
      {
        cout << poses[i] << endl;
        cout << endl;
      }

      for (size_t i = 0; i < controls.size(); ++i)
      {
        cout << controls[i].transpose() << endl;
        cout << endl;
      }

      // if has initial traj, then just use that T
      T = poses.size() - 1;
      this->entries.clear();
      this->finals.clear();

      this->init_trajs.clear();
      vector<Vector6d> traj;
      for (size_t i = 0; i < poses.size(); ++i)
        traj.push_back(logDown(poses[i]));
      this->init_trajs.push_back(traj);
      this->init_controls.push_back(controls);

      this->entries.push_back(this->init_trajs[0].back());
      this->finals.push_back(this->init_trajs[0][0]);

      cout << this->entries[0].transpose() << endl;
      cout << this->finals[0].transpose() << endl;

      this->n_needles = 1;
      this->sim_in_collision = vector<bool>(this->n_needles, false);
      this->distance_to_final = vector<double>(this->n_needles, 0);
    }



    for (int i = 0; i < n_needles; ++i) {
      this->Ts.push_back(T);
    }

    if (!(entry_position_error_relax_x.size() == entry_position_error_relax_y.size() && entry_position_error_relax_y.size() == entry_position_error_relax_z.size())) {
      throw std::runtime_error("entry position error relaxes must have the same size.");
    }

    if (entry_position_error_relax_x.size() > 0) {
      this->entry_position_error_relax.clear();
      for (int i = 0; i < entry_position_error_relax_x.size(); ++i) {
        this->entry_position_error_relax.push_back(Vector3d(entry_position_error_relax_x[i],
                                                            entry_position_error_relax_y[i],
                                                            entry_position_error_relax_z[i]));
      }
    }

    trajopt::SetUserData(*env, "trajopt_cc_hash", CollisionHashPtr(new NeedleCollisionHash(helper)));
  }

  vector<VectorXd> NeedleProblemPlanner::Solve(const vector<VectorXd>& initial) {
    this->current_open_sim_index = 0;
    this->n_multi_iterations = 0;
    vector<KinBodyPtr> robots;
    vector<VectorXd> sols = initial;
    vector<VectorXd> prevsols = initial;
    vector< vector<Vector6d> > states;
    for (int i = 0; i < n_needles; ++i) {
      states.push_back(vector<Vector6d>());
    }
    if (sols.size() == 0) {
      for (int i = 0; i < n_needles; ++i) {
        sols.push_back(VectorXd::Zero(0));
      }
    }
    bool all_converged = false;
    vector<bool> prev_converged(n_needles, true);
    int sequential_solves = 0;
    if (this->is_first_needle_run && (this->separate_planning_first || !this->simultaneous_planning)) {
      while (!all_converged && sequential_solves++ < max_sequential_solves) {
        cout << sequential_solves << endl;
        ++this->n_multi_iterations;
        all_converged = true;
        for (int i = 0; i < n_needles; ++i) {
          trajopt::SetUserData(*this->env, "trajopt_cc", OpenRAVE::UserDataPtr());
          this->managed_kinbodies.clear();
          
          helper->InitParametersFromConsole(this->argc, this->argv);
          if (current_sim_index > 0 && this->use_colocation_correction) {
            // use colocation whenever sim index > 0 
            helper->method = NeedleProblemHelper::Colocation;
          }
          helper->n_needles = 1;

          helper->entries.push_back(this->entries[i]);
          helper->finals.push_back(this->finals[i]);
          helper->init_trajs.push_back(this->init_trajs[i]);
          helper->init_controls.push_back(this->init_controls[i]);
          helper->use_init_traj = this->use_init_traj;

          helper->entry_position_error_relax.push_back(this->entry_position_error_relax[i]);
          helper->entry_orientation_error_relax.push_back(this->entry_orientation_error_relax[i]);
          helper->final_distance_error_relax.push_back(this->final_distance_error_relax[i]);
          helper->Ts.push_back(this->Ts[i]);
          helper->robots.push_back(this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path));
          
          this->env->Add(helper->robots.back(), true);
          /*
          if (!this->is_first_needle_run && i == 0) { // fix entry position if not first run
            helper->entry_position_error_relax.front() = Vector3d::Zero();
            helper->entry_orientation_error_relax.front() = 0;
          }
          */
          for (int j = 0; j < states.size(); ++j) {
            if (i != j){// && prev_converged[j]) {
              this->AddSimulatedNeedleToBullet(states[j]);
            }
          }
          OptProbPtr prob(new OptProb());
          helper->ConfigureProblem(*prob);
          OptimizerT opt(prob);
          helper->ConfigureOptimizer(opt);

          if (sols[i].size() > 0) {
            vector<VectorXd> subinitial;
            subinitial.push_back(sols[i]);
            for (int j = 0; j < subinitial[0].size(); ++j) {
              if (!prev_converged[i]) {
                if (this->channel_planning) subinitial[0](j) += normal() * 0.02; // the environment is less delicate, allowing for more noise
                else subinitial[0](j) += normal() * 0.05;
              }
            }
            helper->SetSolutions(subinitial, opt);
          }


          if (this->stage_plotting || this->stage_result_plotting) {
            this->plotter.reset(new Needle::TrajPlotter());
          }
          if (this->stage_plotting) {
            opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(this->plotter), _1, _2, helper, shared_from_this(), true, vector< vector<Vector6d> >()));
          }

          OptStatus status = opt.optimize();
          if (status != OPT_CONVERGED) {
            cout << "Needle " << i << " not converged" << endl;
            all_converged = false;
          } else {
            cout << "Needle " << i << " converged" << endl;
          }

          prev_converged[i] = status == OPT_CONVERGED;
          for (int j = 0; j < helper->robots.size(); ++j) {
            robots.push_back(helper->robots[j]);
          }
          sols[i] = helper->GetSolutions(opt).front();
          if (this->seq_result_plotting || (this->stage_result_plotting && i == n_needles - 1 && (all_converged || (sequential_solves+1 >= max_sequential_solves)) && !(this->simultaneous_planning))) {
            vector< vector<Vector6d> > plot_states;
            for (int j = 0; j < states.size(); ++j) {
              if (i != j) {
                plot_states.push_back(states[j]);
              }
            }
            this->plotter->OptimizerCallback(prob.get(), opt.x(), this->helper, shared_from_this(), true, plot_states);
          }
          states[i] = helper->GetStates(opt).front();
        }
        if (this->simultaneous_planning) {
          break;
        }
      }
    }

    current_converged = all_converged;

    if (this->channel_planning && !this->simultaneous_planning) {
      //if (all_converged) {
      //  cout << "channel planning converged" << endl;
      //} else {
      //  cout << "channel planning did not converge" << endl;
      //}
      return sols;
    }
    /*
    trajopt::SetUserData(*env, "trajopt_cc", OpenRAVE::UserDataPtr());
    helper->InitParametersFromConsole(this->argc, this->argv);
    if (current_sim_index > 0 && this->use_colocation_correction) {
      // use colocation whenever sim index > 0 
      helper->method = NeedleProblemHelper::Colocation;
    }
    helper->n_needles = this->n_needles;
    helper->entries = this->entries;
    helper->finals = this->finals;
    helper->entry_position_error_relax = this->entry_position_error_relax;
    helper->entry_orientation_error_relax = this->entry_orientation_error_relax;
    helper->final_distance_error_relax = this->final_distance_error_relax;
    helper->Ts = this->Ts;

    if (helper->Ts.front() == 1) {
      helper->trust_box_size = .01;
    }
    if (this->deviation > 0.01) {
      helper->merit_error_coeff = 10;
    } else {
      helper->merit_error_coeff = 100;
      helper->max_merit_coeff_increases -= 2;
    }

    if (!this->is_first_needle_run) { // fix entry position if not first run
      helper->entry_position_error_relax.front() = Vector3d::Zero();
      helper->entry_orientation_error_relax.front() = 0;
    }

    for (int i = 0; i < n_needles; ++i) {
      helper->robots.push_back(this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path));
      this->env->Add(helper->robots.back(), true);
    }

    OptProbPtr prob(new OptProb());
    helper->ConfigureProblem(*prob);
    OptimizerT opt(prob);
    helper->ConfigureOptimizer(opt);

    if (this->is_first_needle_run && (this->separate_planning_first || !this->simultaneous_planning)) {
      helper->SetSolutions(sols, opt);
    } else {
      if (initial.size() == helper->n_needles) {
        helper->SetSolutions(initial, opt);
      } else {
        vector<VectorXd> cursols = helper->GetSolutions(opt);
        for (int i = 0; i < cursols.size(); ++i) {
          for (int j = 0; j < cursols[i].size(); ++j) {
            cursols[i](j) += normal() * 0.05;
          }
        }
        helper->SetSolutions(cursols, opt);
      }
    }

    //if (!this->is_first_needle_run) {
    //  helper->IntegrateControls(opt.x());
    //}

    if (this->stage_plotting || this->stage_result_plotting) {
      this->plotter.reset(new Needle::TrajPlotter());
    }
    if (this->stage_plotting) {
      opt.addCallback(boost::bind(&Needle::TrajPlotter::OptimizerCallback, boost::ref(this->plotter), _1, _2, helper, shared_from_this(), true, vector< vector<Vector6d> >()));
    }

    if (this->simultaneous_planning || !this->is_first_needle_run) {
      //helper->IntegrateControls(opt.x());
      OptStatus status = opt.optimize();
      if (this->stage_result_plotting) {
        this->plotter->OptimizerCallback(prob.get(), this->x, this->helper, shared_from_this(), true, vector< vector<Vector6d> >());
      }
      sols = helper->GetSolutions(opt);
      current_converged = status == OPT_CONVERGED;
    }

    //if (!current_converged && prevsols.size() > 0) {
    //  helper->SetSolutions(prevsols, opt);
    //}

    this->x = opt.x();

    return sols;
    */
  }

  Vector6d NeedleProblemPlanner::PerturbState(const Vector6d& state) {
    Vector6d ret = state;
    for (int i = 0; i < 3; ++i) {
      ret(i) += normal() * 0.05 / 4 * noise_scale;
    }
    for (int i = 3; i < 6; ++i) {
      ret(i) += normal() * 0.025 / 4 * noise_scale;
    }
    return ret;
  }

  void NeedleProblemPlanner::SimulateExecution() {

    
    double phi = helper->GetPhi(this->x, current_open_sim_index, helper->pis.front());
    double Delta = helper->GetDelta(this->x, current_open_sim_index, helper->pis.front());
    double curvature = helper->GetCurvature(this->x, current_open_sim_index, helper->pis.front());


    //cout << "phi: " << phi << endl;
    //cout << "Delta: " << Delta << endl;
    //cout << "curvature: " << curvature << endl;

    if (this->is_first_needle_run) {
      simulated_needle_trajectories.push_back(vector<Vector6d>());
      simulated_needle_trajectories.back().push_back(logDown(helper->pis.front()->local_configs.front()->pose));
    }
    Vector6d state_to_change = simulated_needle_trajectories.back().back();

    Vector6d new_state_without_noise = logDown(helper->TransformPose(expUp(state_to_change), phi, Delta, curvature));
    cout << "perturbing state" << endl;
    Vector6d new_state = PerturbState(new_state_without_noise);
    this->deviation = (new_state_without_noise - new_state).norm();

    simulated_needle_trajectories.back().push_back(new_state);

    this->entries.front() = new_state;

    vector<ConfigurationPtr> rads;
    //rads.push_back(ConfigurationPtr(new LocalConfiguration(helper->pis.front()->local_configs.front()->body, expUp(state_to_change))));
    //rads.push_back(ConfigurationPtr(new LocalConfiguration(helper->pis.front()->local_configs.front()->body, expUp(new_state))));
    
    rads.push_back(helper->pis.front()->local_configs[current_open_sim_index]); rads.push_back(helper->pis.front()->local_configs[current_open_sim_index+1]); 
    TrajArray traj(2, 6); traj.row(0) = logDown(helper->pis.front()->local_configs[current_open_sim_index]->pose.inverse() * expUp(state_to_change));
                          traj.row(1) = logDown(helper->pis.front()->local_configs[current_open_sim_index+1]->pose.inverse() * expUp(new_state));
    vector<Collision> collisions;

    helper->InitializeCollisionEnvironment();

    double old_dis = CollisionChecker::GetOrCreate(*this->env)->GetContactDistance();
    CollisionChecker::GetOrCreate(*this->env)->SetContactDistance(0);//ContinuousCheckTrajectory(traj, rads, collisions);
    //CollisionChecker::GetOrCreate(*this->env)->ContinuousCheckTrajectory(traj, rads, collisions);
    vector<KinBody::LinkPtr> links;
    vector<int> inds;
    rads[0]->GetAffectedLinks(links, true, inds);
    CollisionChecker::GetOrCreate(*this->env)->CastVsAll(*rads[0], *rads[1], links, toDblVec(traj.row(0)), toDblVec(traj.row(1)), collisions);
    CollisionChecker::GetOrCreate(*this->env)->SetContactDistance(old_dis);//ContinuousCheckTrajectory(traj, rads, collisions);
    //boost::shared_ptr<CastCollisionEvaluator> cc(new CastCollisionEvaluator(rads[0], rads[1], )helper->pis.front()->twistvars.row(current_open_sim_index), helper->pis.front()->
    //  helper->pis.front()->local_configs[current_open_sim_index],helper->pis.front()->local_configs[current_open_sim_index+1]

    bool in_collision = collisions.size() > 0;

    if (in_collision) {
      sim_in_collision[current_sim_index] = true;
    }
    if (Ts.front() <= 1 || in_collision) {
      // get rid of first needle
      this->distance_to_final[current_sim_index] =
        (expUp(this->finals[0]).topRightCorner<3, 1>() -
        expUp(new_state).topRightCorner<3, 1>()).norm();
      this->Ts.erase(this->Ts.begin());
      this->entries.erase(this->entries.begin());
      this->finals.erase(this->finals.begin());
      this->entry_position_error_relax.erase(this->entry_position_error_relax.begin());
      this->entry_orientation_error_relax.erase(this->entry_orientation_error_relax.begin());
      this->final_distance_error_relax.erase(this->final_distance_error_relax.begin());
      this->is_first_needle_run = true;
      // pull out the needle and cancel current operation if in collision
      if (!in_collision) {
        this->AddSimulatedNeedleToBullet(this->simulated_needle_trajectories.back());
      }
      --n_needles;
      ++current_sim_index;
    } else {
      --Ts.front();
      this->is_first_needle_run = false;
    }
    ++current_open_sim_index;

  }

  vector<VectorXd> NeedleProblemPlanner::GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol) {
    if (sol.size() > 0) {
      return helper->GetSolutionsWithoutFirstTimestep(sol);
    } else {
      return sol;
    }
  }

  bool NeedleProblemPlanner::Finished() const {
    return this->Ts.size() == 0;
  }

  NeedleProblemPlanner::~NeedleProblemPlanner() {
    RaveDestroy();
  }

  void NeedleProblemPlanner::AddSimulatedNeedleToBullet(const vector<Vector6d>& states) {
    if (states.size() == 0) {
      return;
    }
    KinBodyPtr robot = this->env->ReadRobotURI(RobotBasePtr(), this->robot_file_path);
    this->managed_kinbodies.push_back(robot);
    this->env->Add(robot, true);
    boost::shared_ptr<BulletCollisionChecker> cc = boost::dynamic_pointer_cast<BulletCollisionChecker>(CollisionChecker::GetOrCreate(*this->env));

    vector<LocalConfigurationPtr> local_configs;
    for (int i = 0; i < states.size(); ++i) {
      local_configs.push_back(LocalConfigurationPtr(new LocalConfiguration(robot)));
      local_configs.back()->pose = expUp(states[i]);
      this->managed_configs.push_back(local_configs.back());
    }
    for (int i = 0; i < (int) local_configs.size() - 1; ++i) {
      vector<KinBody::LinkPtr> links;
      vector<int> inds;
      local_configs[i]->GetAffectedLinks(links, true, inds);
      cc->AddCastHullShape(*local_configs[i], *local_configs[i+1], links, toDblVec(Vector6d::Zero()), toDblVec(Vector6d::Zero()));
    }
    this->env->Remove(robot);
    cc->SetLinkIndices();
  }
}
