#ifndef PLANNER_H
#define PLANNER_H

// standard includes
#include <moveit_msgs/MotionPlanRequest.h>
#include <string>
#include <unordered_map>  // TODO: remove
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
#include <sym_plan_msgs/ProcessAttachedCollisionObject.h>  // TODO:
#include <sym_plan_msgs/ProcessCollisionObject.h>          // remove
#include <sym_plan_msgs/RequestIK.h>                       // dependency
#include <sym_plan_msgs/RequestPlan.h>                     // to these

// project includes
#include "collision_space_scene_multithread.h"

namespace symplan {
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
          std::string const & problems_dir
        );
        ~Planner();

        bool Init();
        void RunPlannerServiceServer();
        bool RequestPlanCallback(
          sym_plan_msgs::RequestPlan::Request & request,
          sym_plan_msgs::RequestPlan::Response & response
        );
        bool ProcessCollisionObjectCallback(
          sym_plan_msgs::ProcessCollisionObject::Request & request,
          sym_plan_msgs::ProcessCollisionObject::Response & response
        );
        bool ProcessAttachedCollisionObjectCallback(
          sym_plan_msgs::ProcessAttachedCollisionObject::Request & request,
          sym_plan_msgs::ProcessAttachedCollisionObject::Response & response
        );
        Eigen::Affine3d ComputeFK(std::vector<double> const & joints);
        bool ComputeIK(
          Eigen::Affine3d const & pose,
          std::vector<double> & solution,
          std::vector<double> const & seed_joints,
          int num_retrials = 100
        );
        bool CheckStartState(
          std::vector<double> const & start_state_joints,
          std::vector<double> const & goal,
          std::string const & goal_type = "pose"
        );
        void VisualizeCollisionWorld();
        void VisualizePath(moveit_msgs::RobotTrajectory trajectory, bool repeat = true);
        ros::NodeHandle & GetNh();
        ros::NodeHandle & GetPh();
        std::string GetRobot();
        std::unordered_map<std::string, double> GetConfig();

        constexpr static auto planner_service_name = "planner_service";
        constexpr static auto collision_object_service_name = "collision_object_service";
        constexpr static auto request_ik_service_name = "request_ik_service";

      private:
        bool readProblemParams(
          std::string const & problems_dir,
          std::string const & problem_suffix,
          moveit_msgs::PlanningScene & scene_msg,
          moveit_msgs::MotionPlanRequest & request_msg
        );
        bool requestIKCallback(
          sym_plan_msgs::RequestIK::Request & request,
          sym_plan_msgs::RequestIK::Response & response
        );
        bool setupRobotModel(std::string const & urdf, RobotModelConfig const & config);
        bool readPlannerConfig(ros::NodeHandle const & nh, PlannerConfig & config);
        bool readRobotModelConfig(ros::NodeHandle const & nh, RobotModelConfig & config);
        bool readInitialConfiguration(ros::NodeHandle & nh, moveit_msgs::RobotState & state);
        bool setStartState(double const * state);
        bool setPlanningAndCollisionReferenceState(moveit_msgs::RobotState & state);
        bool fillGoalConstraint(
          std::vector<double> const & pose,
          std::string const & frame_id,
          moveit_msgs::Constraints & goals,
          std::string const & goal_type
        );
        moveit_msgs::CollisionObject getCollisionCube(
          geometry_msgs::Pose const & pose,
          std::vector<double> & dims,
          std::string const & frame_id,
          std::string const & id
        );
        std::vector<moveit_msgs::CollisionObject> getCollisionCubes(
          std::vector<std::vector<double>> & objects,
          std::vector<std::string> & object_ids,
          std::string const & frame_id
        );
        std::vector<moveit_msgs::CollisionObject>
        getCollisionObjects(std::string const & filename, std::string const & frame_id);

        std::shared_ptr<smpl::KDLRobotModel> rm_;
        std::string planning_frame_;
        std::vector<std::string> gripper_links_;
        RobotModelConfig robot_config_;
        PlannerConfig planning_config_;
        std::string robot_description_;
        smpl::PlanningParams planner_params_;

        int num_threads_;
        std::string robot_;
        ros::NodeHandle nh_;
        ros::NodeHandle ph_;
        std::string problems_dir_;
        std::string scene_common_params_;
        std::string request_common_params_;
        std::unordered_map<std::string, double> cfg_;

        smpl::VisualizerROS * visualizer_;
        ros::ServiceServer planner_service_server_;
        ros::ServiceServer collision_object_service_server_;
        ros::ServiceServer request_ik_service_server_;

        moveit_msgs::RobotState start_state_;
        std::shared_ptr<smpl::PlannerInterface> planner_interface_;
        CollisionSpaceSceneMultithread cs_scene_;
        smpl::collision::CollisionSpaceMultithread cc_;
        smpl::OccupancyGrid grid_;
        std::vector<smpl::OccupancyGrid *> grid_vec_;
        std::string attached_co_id_;
    };
}  // namespace symplan

#endif
