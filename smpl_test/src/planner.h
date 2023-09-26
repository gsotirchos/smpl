#ifndef PLANNER_H
#define PLANNER_H

// standard includes
#include <moveit_msgs/MotionPlanRequest.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/planner_interface.h>

// project includes
#include "collision_space_scene_multithread.h"

struct RobotModelConfig {
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

struct PlannerConfig {
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

class Planner {
  public:
    Planner(
      ros::NodeHandle const & nh,
      ros::NodeHandle const & ph,
      int problem_index_width = 4
    );
    ~Planner();

    bool loadProblemCommonParams(std::string const & problems_dir);
    bool planForProblem(int problem_index);
    bool ProcessCollisionObjects(
      std::vector<moveit_msgs::CollisionObject> & objects,
      moveit_msgs::CollisionObject::_operation_type operation
    );
    bool prepareCSSceneCollisionObjects(std::vector<moveit_msgs::CollisionObject> & objects);

    // bool ComputeIK(
    //   Eigen::Affine3d const & pose,
    //   std::vector<double> & solution,
    //   std::vector<double> const & seed_joints,
    //   int num_retrials = 100
    // );

    void VisualizeCollisionWorld();
    void VisualizePath(moveit_msgs::RobotTrajectory trajectory, bool repeat = true);

  private:
    template<typename message_T>
    bool loadYamlToMsg(std::string const & problems_dir, int problem_index, message_T & msg);

    bool readRobotModelConfig(
      ros::NodeHandle const & nh,
      moveit_msgs::MotionPlanRequest const & request_msg
    );
    bool setupRobotModel(std::string const & urdf, RobotModelConfig const & config);
    bool setPlanningAndCollisionReferenceState(
      moveit_msgs::RobotState & state,
      std::vector<geometry_msgs::TransformStamped> & transforms
    );
    bool readPlannerConfig(ros::NodeHandle const & nh);
    bool setupPlannerParams(PlannerConfig & config);

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    smpl::VisualizerROS * visualizer_;

    std::string problems_dir_;
    int problem_index_width_;
    std::string planning_algorithm_;
    int num_threads_;
    std::string planning_frame_;

    std::string robot_name_;
    std::string robot_description_;
    RobotModelConfig robot_config_;
    std::vector<std::string> gripper_links_;
    std::shared_ptr<smpl::KDLRobotModel> rm_;

    PlannerConfig planner_config_;
    smpl::PlanningParams planner_params_;

    std::shared_ptr<smpl::PlannerInterface> planner_interface_;
    CollisionSpaceSceneMultithread cs_scene_;
    smpl::collision::CollisionSpaceMultithread cc_;
    smpl::OccupancyGrid grid_;
    std::vector<smpl::OccupancyGrid *> grid_vec_;
    std::vector<moveit_msgs::CollisionObject> collision_objects_;
};

#endif
