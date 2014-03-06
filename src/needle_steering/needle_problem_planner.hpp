#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_helper.hpp"

namespace Needle {
  struct NeedleProblemPlanner : public boost::enable_shared_from_this<NeedleProblemPlanner> {
    int argc;
    char **argv;
    int n_needles;
    int n_channels;

    int max_sequential_solves;
    vector<int> Ts;
    NeedleProblemHelperPtr helper;

    bool stage_plotting;
    bool stage_result_plotting;
    bool verbose;
    bool is_first_needle_run;
    bool separate_planning_first;
    bool simultaneous_planning;
    bool channel_planning;
    bool current_converged;
    bool seq_result_plotting;
    bool perturb_initialization;
    bool use_colocation_correction;
    double env_transparency;
    double deviation;
    double noise_scale;
    bool use_init_traj;
    string data_dir;
    string env_file_path;
    string robot_file_path;
    string init_traj_path;

    double rotation_axis_index; // index of rotation axis of channel (0 --> around 0'th channel's center)

    EnvironmentBasePtr env;
    boost::shared_ptr<TrajPlotter> plotter;

    vector<Vector6d> entries;
    vector<Vector6d> finals;

    vector<vector<Vector6d> > init_trajs;
    vector<vector<VectorXd> > init_controls;

    DblVec x;
    vector<Vector3d> entry_position_error_relax;
    vector<double> entry_orientation_error_relax;
    vector<double> final_distance_error_relax;
    vector<double> distance_to_final;
    vector<KinBodyPtr> managed_kinbodies;
    vector<LocalConfigurationPtr> managed_configs;

    vector< vector<Vector6d> > simulated_needle_trajectories;
    vector<bool> sim_in_collision;
    int current_sim_index;
    int current_open_sim_index;
    int n_multi_iterations;

    NeedleProblemPlanner(int argc, char **argv);
    ~NeedleProblemPlanner();

    VectorXd PerturbSolution(const VectorXd& sol) const;

    Vector6d PerturbState(const Vector6d& state);
    vector<VectorXd> Solve(const vector<VectorXd>& initial=vector<VectorXd>());
    vector<VectorXd> GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol);
    DblVec Solve(const DblVec& x);
    void SimulateExecution();
    void AddSimulatedNeedleToBullet(const vector<Vector6d>& states);
    bool Finished() const;
  };
}
