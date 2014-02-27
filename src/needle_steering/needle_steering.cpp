#include "needle_steering.hpp"
#include "../utils/clock.hpp"

using namespace Needle;

template <class T>
  string vector_to_array_str(const vector<T>& v) {
    std::stringstream s;
    s << "[";
    for (int i = 0; i < v.size(); ++i) {
      s << v[i];
      if (i < (int) v.size() - 1) {
        s << ", ";
      } else {
        s << "]";
      }
    }
    return s.str();
  }

  template <>
  string vector_to_array_str(const vector<Vector6d>& v) {
    std::stringstream s;
    s << "[";
    for (int i = 0; i < v.size(); ++i) {
      s << vector_to_array_str(toDblVec(v[i]));
      if (i < (int) v.size() - 1) {
        s << ", ";
      } else {
        s << "]";
      }
    }
    return s.str();
  }
  template <class T>
  string vector_vector_to_array_str(const vector< vector<T> >& v) {
    std::stringstream s;
    s << "[";
    for (int i = 0; i < v.size(); ++i) {
      s << vector_to_array_str(v[i]);
      if (i < (int) v.size() - 1) {
        s << ", ";
      } else {
        s << "]";
      }
    }
    return s.str();
  }
int main(int argc, char** argv) {

  NeedleProblemPlannerPtr planner(new NeedleProblemPlanner(argc, argv));

  vector<VectorXd> sols;
  vector< vector<Vector6d> > states;
  states.push_back(planner->starts);
  bool sim_plotting = false;
  bool first_run_only = false;
  bool open_loop = false;
  string plot_channel_path;

  int seed = static_cast<unsigned int>(std::time(0));

  {
    Config config;
    config.add(new Parameter<bool>("sim_plotting", &sim_plotting, "sim_plotting"));
    config.add(new Parameter<bool>("first_run_only", &first_run_only, "first_run_only"));
    config.add(new Parameter<bool>("open_loop", &open_loop, "open_loop"));
    config.add(new Parameter<string>("plot_channel_path", &plot_channel_path, "plot_channel_path"));
    config.add(new Parameter<int>("seed", &seed, "seed"));
    CommandParser parser(config);
    parser.read(argc, argv, true);
  }
  srand(seed);

  cout << "seed: " << seed << endl;

  boost::shared_ptr<NeedleSimPlotter> sim_plotter;

  if (sim_plotting) {
    sim_plotter.reset(new NeedleSimPlotter());
  }

  util::StartClock();
  while (!planner->Finished()) {
    sols = planner->Solve(sols);
    if (first_run_only) break;
    while (!planner->Finished()) {
      sols = planner->GetSolutionsWithoutFirstTimestep(sols);
      planner->SimulateExecution();
      if (sim_plotting) {
        sim_plotter->Plot(planner);
      }
      if (!open_loop) break;
    }
  }
  cout << "elapsed time: " << util::GetClock() << endl;

  if (first_run_only) {
    trajopt::SetUserData(*planner->env, "trajopt_cc", OpenRAVE::UserDataPtr());
    NeedleProblemHelperPtr helper = planner->helper;
    helper->InitParametersFromConsole(argc, argv); 
    helper->n_needles = planner->n_needles;
    helper->starts = planner->starts;
    helper->goals = planner->goals;
    helper->start_position_error_relax = planner->start_position_error_relax;
    helper->start_orientation_error_relax = planner->start_orientation_error_relax;
    helper->goal_distance_error_relax = planner->goal_distance_error_relax;
    helper->Ts = planner->Ts;
    for (int i = 0; i < planner->n_needles; ++i) {
      helper->robots.push_back(planner->env->ReadRobotURI(RobotBasePtr(), planner->robot_file_path));
      planner->env->Add(helper->robots.back(), true);
    }
    OptProbPtr prob(new OptProb());
    helper->ConfigureProblem(*prob);
    OptimizerT opt(prob);
    helper->ConfigureOptimizer(opt);
    helper->SetSolutions(sols, opt);
    DblVec& x = opt.x();
    DblVec rotation_costs, speed_costs, clearance_costs;
    for (int i = 0; i < helper->rotation_costs.size(); ++i) {
      rotation_costs.push_back(helper->rotation_costs[i]->value(x, opt.getModel().get()));
    }
    for (int i = 0; i < helper->speed_costs.size(); ++i) {
      speed_costs.push_back(helper->speed_costs[i]->value(x, opt.getModel().get()));
    }
    for (int i = 0; i < helper->clearance_costs.size(); ++i) {
      clearance_costs.push_back(helper->clearance_costs[i]->value(x, opt.getModel().get()));
    }
    vector< vector<Vector6d> > states = helper->GetStates(opt);
    cout << "twist costs: " << vector_to_array_str(rotation_costs) << endl;
    cout << "path length costs: " << vector_to_array_str(speed_costs) << endl;
    cout << "clearance costs: " << vector_to_array_str(clearance_costs) << endl;
    cout << "planned trajectories: " << vector_vector_to_array_str(states) << endl;
    cout << "phis: " << vector_vector_to_array_str(helper->GetPhis(opt)) << endl;
    cout << "Deltas: " << vector_vector_to_array_str(helper->GetDeltas(opt)) << endl;
    cout << "curvatures: " << vector_vector_to_array_str(helper->GetCurvatures(opt)) << endl;
    cout << "n_multi_iterations: " << planner->n_multi_iterations << endl;
    if (planner->current_converged) {
      cout << "status: converged" << endl;
    } else {
      cout << "status: not_converged" << endl;
    }
  } else {

    cout << "collision status: " << vector_to_array_str(planner->sim_in_collision) << endl;
    
    cout << "distance to goals: " << vector_to_array_str(planner->distance_to_goal) << endl;

    cout << "simulated trajectories: " << vector_vector_to_array_str(planner->simulated_needle_trajectories) << endl;
  }

  return 0;

}
