// standard includes
// #include <Eigen/src/Core/MatrixBase.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <smpl/planning_params.h>
#include <string>
#include <thread>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/PlanningScene.h>
#include <robowflex_library/io/yaml.h>
#include <ros/ros.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/planner_interface.h>

// project includes
#include "collision_space_scene_multithread.h"
#include "planner.h"


constexpr bool VERBOSE = true;


// Public

Planner::Planner(
  ros::NodeHandle const & nh,
  ros::NodeHandle const & ph,
  int problem_index_width
) :
  nh_(nh),
  ph_(ph),
  problem_index_width_(problem_index_width) {
    if (VERBOSE) {
        ROS_INFO("Initialize visualizer");
    }
    smpl::VisualizerROS * visualizer_ = new smpl::VisualizerROS(nh_, 100);
    smpl::viz::set_visualizer(visualizer_);
}

Planner::~Planner() {
    delete visualizer_;
}

bool Planner::loadProblemCommonParams(std::string const & problems_dir) {
    problems_dir_ = problems_dir;

    // Read an arbitrary problem's parameters (to get the common parameters)
    int const arbitrary_problem_index = 1;

    moveit_msgs::PlanningScene scene_common_msg;
    if (!loadYamlToMsg(problems_dir_, arbitrary_problem_index, scene_common_msg)) {
        ROS_ERROR(
          "Could not read problem %d's scene message. Are the contents of %s properly formatted?",
          arbitrary_problem_index,
          problems_dir_.c_str()
        );
    }

    moveit_msgs::MotionPlanRequest request_common_msg;
    if (!loadYamlToMsg(problems_dir_, arbitrary_problem_index, request_common_msg)) {
        ROS_ERROR(
          "Could not read problem %d's request message. Are the contents of %s properly formatted?",
          arbitrary_problem_index,
          problems_dir_.c_str()
        );
    }

    if (!ph_.getParam("planning_algorithm", planning_algorithm_)) {
        ROS_ERROR("Failed to retrieve param 'planning_algorithm' from the param server");
        return false;
    }
    if (VERBOSE) {
        ROS_INFO("planning_algorithm: %s \n", planning_algorithm_.c_str());
    }

    if (!ph_.getParam("num_threads", num_threads_)) {
        ROS_ERROR("Failed to retrieve param 'num_threads' from the param server");
        return false;
    }
    if (VERBOSE) {
        ROS_INFO("num_threads: %d \n", num_threads_);
    }

    // Everyone needs to know the name of the planning frame for
    // reasons...
    // ...frame_id for the occupancy grid (for visualization)
    // ...frame_id for collision objects (must be the same as the
    // grid, other than that, useless)
    if (!ph_.getParam("planning_frame", planning_frame_)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return false;
    }

    /////////////////
    // Robot Model //
    /////////////////

    if (VERBOSE) {
        ROS_INFO("Load common parameters");
    }

    robot_name_ = scene_common_msg.robot_model_name;
    if (robot_name_ == "") {
        ROS_ERROR(
          "Failed to retrieve param 'robot_model_name' from the scene configuration file"
        );
        return false;
    }

    // Robot description is required to initialize collision checker and robot model...
    auto const robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh_.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return false;
    }
    if (!nh_.getParam(robot_description_param, robot_description_)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return false;
    }

    if (!readRobotModelConfig(ros::NodeHandle("~robot_model"), request_common_msg)) {
        ROS_ERROR("Failed to read robot model config from the scene configuration file");
        return false;
    }

    if (!setupRobotModel(robot_description_, robot_config_)) {
        ROS_ERROR("Failed to set up Robot Model");
        return false;
    }
    ROS_ERROR("%s \n", rm_->getBaseLink().c_str());

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    if (VERBOSE) {
        ROS_INFO("Initialize Occupancy Grid");
    }

    double df_size_x, df_size_y, df_size_z, df_res, df_origin_x, df_origin_y, df_origin_z,
      max_distance;

    // !!ASSUMPTION: all problems share the same start state
    auto ee_position = rm_->computeFK(request_common_msg.start_state.joint_state.position)
                         .translation();

    df_size_x = ee_position.x() + request_common_msg.workspace_parameters.max_corner.x
                - request_common_msg.workspace_parameters.min_corner.x;
    df_size_y = ee_position.y() + request_common_msg.workspace_parameters.max_corner.y
                - request_common_msg.workspace_parameters.min_corner.y;
    df_size_z = ee_position.z() + request_common_msg.workspace_parameters.max_corner.z
                - request_common_msg.workspace_parameters.min_corner.z;
    df_origin_x = ee_position.x() + request_common_msg.workspace_parameters.min_corner.x;
    df_origin_y = ee_position.y() + request_common_msg.workspace_parameters.min_corner.y;
    df_origin_z = ee_position.z() + request_common_msg.workspace_parameters.min_corner.z;
    df_res = 0.02;       // TODO later:
    max_distance = 1.8;  // set these externally(?)

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
      df_origin_x,
      df_origin_y,
      df_origin_z,
      df_size_x,
      df_size_y,
      df_size_z,
      df_res,
      max_distance
    );

    auto ref_counted = false;
    smpl::OccupancyGrid const grid(df, ref_counted);
    grid_ = grid;

    grid_.setReferenceFrame(planning_frame_);
    SV_SHOW_INFO(grid_.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    if (VERBOSE) {
        ROS_INFO("Initialize collision checker");
    }

    // Load robot-specific spheres collision model
    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph_, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return false;
    }

    if (!cc_.init(
          num_threads_,
          &grid_,
          grid_vec_,
          robot_description_,
          cc_conf,
          robot_config_.group_name,
          robot_config_.planning_joints
        )) {
        ROS_ERROR("Failed to initialize Collision Space");
        return false;
    }

    // Load allowed collision pairs
    smpl::collision::AllowedCollisionMatrix acm;
    auto const link_names = scene_common_msg.allowed_collision_matrix.entry_names;
    auto const collision_values = scene_common_msg.allowed_collision_matrix.entry_values;
    for (unsigned int i = 0; i < link_names.size(); i++) {
        for (unsigned int j = i + 1; j < link_names.size(); j++) {
            if (collision_values[i].enabled[j]) {
                acm.setEntry(link_names[i], link_names[j], true);
            }
        }
    }
    cc_.setAllowedCollisionMatrix(acm);

    /////////////////
    // Setup Scene //
    /////////////////

    if (VERBOSE) {
        ROS_INFO("Initialize scene");
    }

    cs_scene_.SetCollisionSpace(&cc_);

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(
          scene_common_msg.robot_state,
          scene_common_msg.fixed_frame_transforms
        )) {
        ROS_ERROR("Failed to set planning and collision reference state");
        return false;
    }

    // SV_SHOW_INFO(grid_.getDistanceFieldVisualization(0.2));

    ///////////////////
    // Planner Setup //
    ///////////////////

    if (!readPlannerConfig(ros::NodeHandle("~planning"))) {
        ROS_ERROR("Failed to read planner config");
        return false;
    }

    if (!setupPlannerParams(planner_config_)) {
        ROS_ERROR("Failed to set planner parameters");
        return false;
    }

    // Set up planner interface
    // TODO?: This also exists in plan(); which should I keep?
    planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, grid_vec_);
    if (!planner_interface_->init(planner_params_)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return false;
    }

    VisualizeCollisionWorld();

    return true;
}

bool Planner::planForProblem(int problem_index) {
    std::string goal_type;
    if (!ph_.getParam("goal_type", goal_type)) {
        ROS_ERROR("Failed to retrieve param 'goal_type' from the param server");
        return false;
    }

    // Read the specified problem's parameters
    moveit_msgs::MotionPlanRequest request_msg;
    if (!loadYamlToMsg(problems_dir_, problem_index, request_msg)) {
        ROS_ERROR(
          "Could not read problem %d's request message. Are the contents of %s properly formatted?",
          problem_index,
          problems_dir_.c_str()
        );
    }

    moveit_msgs::PlanningScene scene_msg;
    if (!loadYamlToMsg(problems_dir_, problem_index, scene_msg)) {
        ROS_ERROR(
          "Could not read problem %d's scene message. Are the contents of %s properly formatted?",
          problem_index,
          problems_dir_.c_str()
        );
    }

    // Remove all previous collision objects from scene and add those of the current problem
    if (!prepareCSSceneCollisionObjects(scene_msg.world.collision_objects)) {
        ROS_ERROR(
          "Failed to prepare collision space scene with the problem's collision objects"
        );
        return false;
    }

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(
          request_msg.start_state,
          scene_msg.fixed_frame_transforms
        )) {
        ROS_ERROR("Failed to set planning and collision reference state");
        return false;
    }

    // Set planning algorithm id
    if (goal_type == "pose") {
        request_msg.planner_id = planning_algorithm_ + ".bfs.manip";
    } else if (goal_type == "joints") {
        request_msg.planner_id = planning_algorithm_ + ".joint_distance.manip";
    } else {
        ROS_ERROR("Goal type not identified!");
        return false;
    }

    // Set up planner interface
    // TODO?: This also exists in loadProblemCommonParams(); which should I keep?
    // planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, grid_vec_);
    // if (!planner_interface_->init(planner_params_)) {
    //     ROS_ERROR("Failed to initialize Planner Interface");
    //     return false;
    // }
    //===========================================================================

    // plan
    if (VERBOSE) {
        ROS_INFO("Calling solve...");
    }

    moveit_msgs::MotionPlanResponse res;
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = request_msg.start_state;

    auto plan_found = planner_interface_->solve(moveit_msgs::PlanningScene{}, request_msg, res);
    if ((!plan_found) || (res.trajectory.joint_trajectory.points.size() == 0)) {
        ROS_ERROR("Failed to plan");
        return false;
    }

    auto planning_stats = planner_interface_->getPlannerStats();

    if (VERBOSE) {
        ROS_INFO("Planning statistics");
        for (auto & entry : planning_stats) {
            ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
        }
    }

    VisualizePath(res.trajectory);

    return true;
}

bool Planner::ProcessCollisionObjects(
  std::vector<moveit_msgs::CollisionObject> & objects,
  moveit_msgs::CollisionObject::_operation_type operation
) {
    for (auto & object : objects) {
        if (VERBOSE) {
            std::cout << " collision object: " << std::string(object.id) << object.operation
                      << '\n';
        }

        object.header.frame_id = planning_frame_;
        object.operation = operation;

        for (int tidx = 0; tidx < num_threads_; ++tidx) {
            if (!cs_scene_.ProcessCollisionObjectMsg(tidx, object)) {
                if (VERBOSE) {
                    std::cout << "Could not " << object.operation << " collision object "
                              << std::string(object.id) << " for thread " << tidx << '\n';
                    return false;
                }
            } else {
                if (VERBOSE) {
                    std::cout << "Successfully " << object.operation << "ed collision object "
                              << std::string(object.id) << " for thread " << tidx << '\n';
                }
            }
        }
    }

    VisualizeCollisionWorld();

    std::cout << "All collision objects successfully processed!" << '\n';

    return true;
}

bool Planner::prepareCSSceneCollisionObjects(std::vector<moveit_msgs::CollisionObject> & objects
) {
    // Remove any previous collision objects from scene
    if (!ProcessCollisionObjects(collision_objects_, moveit_msgs::CollisionObject::REMOVE)) {
        ROS_ERROR("Failed to remove all previour collision objects from scene");
        return false;
    }

    // Add the current planning problem's collision objects to scene
    if (!ProcessCollisionObjects(objects, moveit_msgs::CollisionObject::ADD)) {
        ROS_ERROR("Failed to add all collision objects to planning scene");
        return false;
    }
    collision_objects_ = objects;

    return true;
}

/*
bool Planner::ComputeIK(
  Eigen::Affine3d const & pose,
  std::vector<double> & solution,
  std::vector<double> const & seed_joints,
  int num_retrials
) {
    bool success = rm_->computeIK(pose, seed_joints, solution, smpl::ik_option::UNRESTRICTED);
    int retry_count = 0;
    while ((!success) && (retry_count < num_retrials)) {
        std::vector<double> seed;
        for (int jidx = 0; jidx < 7; ++jidx) {
            seed.push_back(2 * M_PI * rand_r(reinterpret_cast<unsigned *>(&jidx)));
        }

        success = rm_->computeIK(pose, seed, solution, smpl::ik_option::UNRESTRICTED);
        retry_count++;
    }

    if (!success) {
        return false;
        // ROS_WARN("Failed IK !");
    }
    return true;
}
*/

void Planner::VisualizeCollisionWorld() {
    while (!ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    auto bounding_box = cc_.grid()->getBoundingBoxVisualization();
    auto markers = cc_.getCollisionWorldVisualization(0);
    auto occupied_voxels = cc_.getOccupiedVoxelsVisualization();
    SV_SHOW_INFO(bounding_box);
    SV_SHOW_INFO(markers);
    SV_SHOW_INFO(occupied_voxels);
}

void Planner::VisualizePath(moveit_msgs::RobotTrajectory trajectory, bool repeat) {
    if (VERBOSE) {
        ROS_INFO("Animate path");
    }

    int point_idx = 0;
    while (ros::ok()) {
        auto & point = trajectory.joint_trajectory.points[point_idx];
        auto markers_robot = cc_.getCollisionRobotVisualization(0, point.positions);
        // for (auto & m : markers.markers) {
        //     m.ns = "path_animation";
        // }
        SV_SHOW_INFO(markers_robot);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        point_idx++;
        point_idx %= static_cast<int>(trajectory.joint_trajectory.points.size());

        if (!repeat) {
            break;
        }
    }
}


// Private

template<typename T>
bool Planner::loadYamlToMsg(std::string const & problems_dir, int problem_index, T & msg) {
    std::ostringstream problem_index_ss;
    problem_index_ss << std::setw(problem_index_width_) << std::setfill('0') << problem_index;

    std::string file_name_prefix;
    if (std::is_same<T, moveit_msgs::MotionPlanRequest>::value) {
        file_name_prefix = "/request";
    } else if (std::is_same<T, moveit_msgs::PlanningScene>::value) {
        file_name_prefix = "/scene";
    }
    std::string const yaml_file = problems_dir + "/" + file_name_prefix
                                  + problem_index_ss.str() + ".yaml";

    robowflex::IO::fromYAMLFile(msg, yaml_file);
    return true;
}

bool Planner::readRobotModelConfig(
  ros::NodeHandle const & nh,
  moveit_msgs::MotionPlanRequest const & request_msg
) {
    robot_config_.group_name = request_msg.group_name;
    if (robot_config_.group_name == "") {
        ROS_ERROR("Failed to read 'group_name' from the request robot_config_uration file");
        return false;
    }

    for (auto & joint_constraint : request_msg.goal_constraints[0].joint_constraints) {
        robot_config_.planning_joints.push_back(joint_constraint.joint_name);
    }

    nh.getParam("kinematics_frame", robot_config_.kinematics_frame);
    nh.getParam("chain_tip_link", robot_config_.chain_tip_link);
    return true;
}

bool Planner::setupRobotModel(std::string const & urdf, RobotModelConfig const & config) {
    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR(
          "Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server"
        );
        return false;
    }

    if (VERBOSE) {
        ROS_INFO("Construct Generic KDL Robot Model");
    }
    rm_ = std::make_shared<smpl::KDLRobotModel>();

    if (!rm_->init(urdf, config.kinematics_frame, config.chain_tip_link, num_threads_)) {
        ROS_ERROR("Failed to initialize robot model");
        return false;
    }

    // // To handle smpl bug where IK does not work without one call to FK
    // std::vector<double> const joint_states =
    //   {0.45274857, 0.58038735, -0.11977164, -1.9721714, 0.13661714, 2.5460937, 1.040775};
    //
    // for (int tidx = 0; tidx < num_threads_; ++tidx) {
    //     auto fk_solution = rm_->computeFK(joint_states, tidx);
    // }
    //
    // std::vector<double> solution;
    //
    // bool success = rm_->computeIK(fk_solution, joint_states, solution, smpl::ik_option::UNRESTRICTED);
    // auto ea = fk_solution.rotation().eulerAngles(0, 1, 2);
    // std::cout << "Euler from quaternion in roll, pitch, yaw" << '\n' << ea << '\n';
    // std::cout << "pose " << fk_solution.translation() << '\n';
    return true;
}

bool Planner::setPlanningAndCollisionReferenceState(
  moveit_msgs::RobotState & state,
  std::vector<geometry_msgs::TransformStamped> & transforms
) {
    // Set reference state in the robot planning model...
    smpl::urdf::RobotState reference_state;
    InitRobotState(&reference_state, &rm_->m_robot_model);
    for (unsigned int i = 0; i < state.joint_state.name.size(); ++i) {
        auto * var = GetVariable(&rm_->m_robot_model, &state.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN(
              "Failed get the robot model's joint variable for \"%s\"",
              state.joint_state.name[i].c_str()
            );
            continue;
        }
        if (VERBOSE) {
            ROS_INFO(
              "Set joint %s to %f",
              state.joint_state.name[i].c_str(),
              state.joint_state.position[i]
            );
        }
        SetVariablePosition(&reference_state, var, state.joint_state.position[i]);
    }
    SetReferenceState(rm_.get(), GetVariablePositions(&reference_state));

    // Get the world to model transform
    Eigen::Affine3d worldToModelTransform;
    for (auto & transform : transforms) {
        if ((state.joint_state.header.frame_id == "" && transform.child_frame_id != "world")
            || (state.joint_state.header.frame_id != transform.child_frame_id)) {
            continue;
        }

        tf::transformMsgToEigen(transform.transform, worldToModelTransform);
        break;
    }

    // Set reference state in the collision model...
    for (auto tidx = 0; tidx < num_threads_; ++tidx) {
        if (!cs_scene_.SetRobotState(tidx, state)) {
            ROS_ERROR("Failed to set start state on Collision Space Scene");
            return false;
        }

        cc_.setWorldToModelTransform(tidx, worldToModelTransform);
    }

    return true;
}

bool Planner::readPlannerConfig(ros::NodeHandle const & nh) {
    if (!nh.getParam("discretization", planner_config_.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", planner_config_.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", planner_config_.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", planner_config_.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", planner_config_.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", planner_config_.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", planner_config_.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", planner_config_.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", planner_config_.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", planner_config_.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

bool Planner::setupPlannerParams(PlannerConfig & config) {
    planner_params_.addParam("num_threads", num_threads_);
    planner_params_.addParam("discretization", config.discretization);
    planner_params_.addParam("mprim_filename", config.mprim_filename);
    planner_params_.addParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim);
    planner_params_.addParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim);
    planner_params_.addParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim);
    planner_params_.addParam("use_short_dist_mprims", config.use_short_dist_mprims);
    planner_params_.addParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh);
    planner_params_.addParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh);
    planner_params_.addParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh);
    planner_params_.addParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh);
    planner_params_.addParam("epsilon", 100.0);
    planner_params_.addParam("search_mode", false);
    planner_params_.addParam("allow_partial_solutions", false);
    planner_params_.addParam("target_epsilon", 1.0);
    planner_params_.addParam("delta_epsilon", 1.0);
    planner_params_.addParam("improve_solution", false);
    planner_params_.addParam("bound_expansions", true);
    planner_params_.addParam("repair_time", 1.0);
    planner_params_.addParam("bfs_inlation_radius", 0.02);
    planner_params_.addParam("bfs_cost_per_cell", 100);

    return true;
}
