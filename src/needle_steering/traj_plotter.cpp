#include "traj_plotter.hpp"
#include "utils.hpp"

namespace Needle {

  TrajPlotter::TrajPlotter() {}

  bool TrajPlotter::OptimizerCallback(OptProb* prob, DblVec& x, NeedleProblemHelperPtr helper, NeedleProblemPlannerPtr planner, bool halt = true, const vector< vector<Vector6d> >& extra_states = vector< vector<Vector6d> >()) {
    vector<GraphHandlePtr> handles;
    OSGViewerPtr viewer = OSGViewer::GetOrCreate(helper->pis[0]->local_configs[0]->GetEnv());
    //BOOST_FOREACH(CostPtr& cost, prob->getCosts()) {
    //  if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
    //    plotter->Plot(x, *(helper->local_configs[0]->GetEnv()), handles);
    //  }
    //}
    //vector<ConstraintPtr> constraints = prob->getConstraints();
    //BOOST_FOREACH(ConstraintPtr& cnt, constraints) {
    //  if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
    //    plotter->Plot(x, *(helper->local_configs[0]->GetEnv()), handles);
    //  }
    //}
    EnvironmentBasePtr env = helper->pis[0]->local_configs[0]->GetEnv();
    
    //CollisionChecker::GetOrCreate(*env)->PlotCollisionGeometry(handles);//SetContactDistance(collision_dist_pen + 0.05);
    viewer->SetAllTransparency(0.1);
    for (int k = 0; k < helper->pis.size(); ++k) {
      vector<KinBodyPtr> bodies = helper->pis[k]->local_configs[0]->GetBodies();
      MatrixXd vals = getTraj(x, helper->pis[k]->twistvars);
      for (int i=0; i < vals.rows(); ++i) {
        helper->pis[k]->local_configs[i]->SetDOFValues(toDblVec(vals.row(i)));
        BOOST_FOREACH(const KinBodyPtr& body, bodies) {
          handles.push_back(viewer->PlotKinBody(body));
          SetTransparency(handles.back(), 1);
        }
      }
    }
    KinBodyPtr robot = env->ReadRobotURI(RobotBasePtr(), planner->robot_file_path);
    planner->env->Add(robot, true);
    for (int i = 0; i < extra_states.size(); ++i) {
      for (int j = 0; j < extra_states[i].size(); ++j) {
        robot->SetTransform(matrixToTransform(expUp(extra_states[i][j])));
        handles.push_back(viewer->PlotKinBody(robot));
        SetTransparency(handles.back(), 0.35);
      }
    }
    planner->env->Remove(robot);
    if (halt) viewer->Idle();
    return false;
  }

  NeedleSimPlotter::NeedleSimPlotter() {}

  void NeedleSimPlotter::Plot(NeedleProblemPlannerPtr planner) {
    KinBodyPtr robot = planner->env->ReadRobotURI(RobotBasePtr(), planner->robot_file_path);
    planner->env->Add(robot, true);
    vector<GraphHandlePtr> handles;
    OSGViewerPtr viewer = OSGViewer::GetOrCreate(planner->env);
    viewer->UpdateSceneData();
    viewer->SetAllTransparency(planner->env_transparency);
    for (int i = 0; i < planner->simulated_needle_trajectories.size(); ++i) {
      for (int j = 0; j < planner->simulated_needle_trajectories[i].size(); ++j) {
        robot->SetTransform(matrixToTransform(expUp(planner->simulated_needle_trajectories[i][j])));
        handles.push_back(viewer->PlotKinBody(robot));
        SetTransparency(handles.back(), 1);
      }
    }
    planner->env->Remove(robot);
    viewer->Idle();
  }
}
