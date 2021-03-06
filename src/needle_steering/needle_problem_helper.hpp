#pragma once

#include "common.hpp"
#include "fwd.hpp"
#include "needle_problem_instance.hpp"

namespace Needle {
  struct NeedleProblemHelper : public boost::enable_shared_from_this<NeedleProblemHelper> {
    enum Method { Colocation = 1, Shooting = 2 };
    enum RotationCost { UseRotationQuadraticCost = 1, UseRotationL1Cost = 2 };
    // Config parameters
    vector<Vector6d> entries;
    vector<Vector6d> finals;
    vector<vector<Vector6d> > init_trajs;
    vector<vector<VectorXd> > init_controls;
    bool use_init_traj;
    int n_needles;
    double coeff_rotation;
    double coeff_rotation_regularization;
    double coeff_curvature;
    double coeff_rotation_continuity;
    double coeff_rotation_continuity_regularization;
    double coeff_curvature_continuity;

    int max_iter;

    double coeff_speed;
    double coeff_orientation_error;
    double improve_ratio_threshold;
    double trust_shrink_ratio;
    double trust_expand_ratio;
    double merit_error_coeff;
    int max_merit_coeff_increases;
    double trust_box_size;
    bool record_trust_region_history;
    double cnt_tolerance;
    vector<int> Ts;
    int n_dof;
    int method;
    int rotation_cost;
    bool use_collision_clearance_cost;
    bool verbose;
    bool continuous_collision;
    bool channel_planning;
    bool channel_continuity;

    double env_transparency;
    double r_min;
    vector<string> ignored_kinbody_names;
    double collision_dist_pen;
    double collision_coeff;
    double collision_clearance_coeff;
    double collision_clearance_threshold;

    double total_curvature_limit;
    double total_rotation_limit;
    double rotation_bound;

    double channel_environment_radius;
    double channel_environment_height;
    vector<KinBodyPtr> robots;

    vector<NeedleProblemInstancePtr> pis;
    vector<ConstraintPtr> self_collision_constraints;

    vector<CostPtr> rotation_costs;
    vector<CostPtr> curvature_costs;
    vector<CostPtr> speed_costs;
    vector<CostPtr> clearance_costs;

    vector<Vector3d> entry_position_error_relax;
    vector<double> entry_orientation_error_relax;
    vector<double> final_distance_error_relax;

    void ConfigureProblem(OptProb& prob);
    void InitOptimizeVariables(OptimizerT& opt);
    bool OptimizerCallback(OptProb*, DblVec& x);
    void ConfigureOptimizer(OptimizerT& opt);
    vector<VectorXd> GetSolutionsWithoutFirstTimestep(const vector<VectorXd>& sol);



    void InitParameters();
    void InitParametersFromConsole(int argc, char** argv);
    void Clear();

    void CreateVariables(OptProb& prob, NeedleProblemInstancePtr pi);
    void InitLocalConfigurations(const KinBodyPtr robot, OptProb& prob, NeedleProblemInstancePtr pi);
    void InitTrajectory(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddRotationCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddCurvatureCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddRotationContinuityCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddCurvatureContinuityCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddRotationConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddCurvatureConstraint(OptProb& prob, NeedleProblemInstancePtr helper);


    void AddSpeedCost(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddChannelConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddEntryConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddControlConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddLinearizedControlConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr pi);
    void AddSelfCollisionConstraint(OptProb& prob, NeedleProblemInstancePtr piA, NeedleProblemInstancePtr piB);
    void AddCollisionClearanceCost(OptProb& prob);
    void InitializeCollisionEnvironment();



    Matrix4d TransformPose(const Matrix4d& pose, double phi, double Delta, double radius) const;
    double GetPhi(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetDelta(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;
    double GetCurvature(const DblVec& x, int i, NeedleProblemInstancePtr pi) const;

    vector<VectorXd> GetSolutions(OptimizerT& opt);
    vector< vector<Vector6d> > GetStates(OptimizerT& opt);
    void SetSolutions(const vector<VectorXd>& sol, OptimizerT& opt);
    void IntegrateControls(DblVec& x);
    void IntegrateControls(DblVec& x, size_t i);


    void AddNeedlesToBullet(OptimizerT& prob);
    void AddNeedleToBullet(NeedleProblemInstancePtr pi, OptimizerT& prob);

    vector<DblVec> GetPhis(OptimizerT& opt);
    vector<DblVec> GetDeltas(OptimizerT& opt);
    vector<DblVec> GetCurvatures(OptimizerT& opt);

    #ifdef NEEDLE_TEST
    void checkAlignment(DblVec& x);
    #endif
  };

}
