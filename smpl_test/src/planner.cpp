// standard includes
#include <boost/filesystem.hpp>
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


// Public //

Planner::Planner(
  ros::NodeHandle const & nh,
  ros::NodeHandle const & ph,
  bool verbose,
  bool visualize
) :
  nh_(nh),
  ph_(ph),
  verbose_(verbose),
  visualize_(visualize) {
    separator_ = ", ";
    file_suffix_ = ".csv";
    stats_file_prefix_ = "benchmarking_smpl/";

    if (verbose_) {
        ROS_INFO("Initialize visualizer");
    }

    // Wait for an RViz instance (TODO: possibly unnecessary)
    while (visualize_ && !rvizIsRunning()) {
        if (verbose_) {
            ROS_INFO("Waiting for RViz instance to start before instantiating a planner...");
        }
        ros::Duration(0.5).sleep();
    }

    visualizer_ = new smpl::VisualizerROS(nh_, 100);
    smpl::viz::set_visualizer(visualizer_);
}

Planner::~Planner() {
    if (stats_file_.is_open()) {
        stats_file_.close();
    }

    delete visualizer_;
}

// TODO: possibly unnecessary
bool Planner::rvizIsRunning() {
    ros::V_string node_names;

    if (!ros::master::getNodes(node_names)) {
        ROS_WARN("Failed to get nodes names. Is roscore running?");
        return false;
    }

    if (!std::count(node_names.begin(), node_names.end(), "/rviz")) {
        return false;
    }

    return true;
}

bool Planner::initForProblemsDir(std::string const & problems_dir) {
    problems_dir_ = problems_dir;

    // Check whether the specified problems directory exists
    auto problems_fs = boost::filesystem::path(problems_dir);
    if (!boost::filesystem::is_directory(problems_fs)) {
        ROS_ERROR(
          "Failed to locate a proper problems directory in the specified path: %s",
          problems_dir_.c_str()
        );
        return 1;
    }

    // Delete all existing planning stats files in ~/.ros/benchmarking_smpl
    boost::filesystem::remove_all(boost::filesystem::path(stats_file_prefix_));
    boost::filesystem::create_directory(boost::filesystem::path(stats_file_prefix_));

    std::string const problem_name = problems_fs.has_filename() ?
                                       problems_fs.filename().string() :
                                       problems_fs.parent_path().string();

    // Open a planning stats file handle for ~/.ros/benchmarking_smpl/<problem_name>.csv
    std::string const stats_file_path = stats_file_prefix_ + "/" + problem_name + file_suffix_;
    stats_file_.open(stats_file_path);
    if (verbose_) {
        ROS_INFO("Open new results file: %s", stats_file_path.c_str());
    }

    stats_file_
      << "problem index" << separator_ << "edge_expansions" << separator_ << "expansions"
      << separator_ << "final epsilon" << separator_ << "final epsilon planning time"
      << separator_ << "initial epsilon" << separator_ << "initial solution expansions"
      << separator_ << "initial solution planning time" << separator_ << "solution cost"
      << separator_ << "solution epsilon" << separator_ << "state_expansions" << separator_
      << "time" << std::endl;

    // Read the problems' index width (e.g. scene0007.yaml -> 4)
    if (!ph_.getParam("planning_problem_index_width", problem_index_width_)) {
        ROS_ERROR(
          "Failed to retrieve param 'planning_problem_index_width' from the param server"
        );
        return false;
    }

    if (!ph_.getParam("planning_algorithm", planning_algorithm_)) {
        ROS_ERROR("Failed to retrieve param 'planning_algorithm' from the param server");
        return false;
    }
    if (verbose_) {
        ROS_INFO("planning_algorithm: %s \n", planning_algorithm_.c_str());
    }

    if (!ph_.getParam("goal_type", goal_type_)) {
        ROS_ERROR("Failed to retrieve param 'goal_type' from the param server");
        return false;
    }

    if (!ph_.getParam("num_threads", num_threads_)) {
        ROS_ERROR("Failed to retrieve param 'num_threads' from the param server");
        return false;
    }
    if (verbose_) {
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

    if (verbose_) {
        ROS_INFO("Load common parameters");
    }

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

    robot_name_ = scene_common_msg.robot_model_name;
    if (robot_name_ == "") {
        ROS_ERROR("Failed to read 'robot_model_name' from the scene configuration file");
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
        ROS_ERROR("Failed to set up robot model");
        return false;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    if (verbose_) {
        ROS_INFO("Initialize Occupancy Grid");
    }

    double df_size_x, df_size_y, df_size_z, df_res, df_origin_x, df_origin_y, df_origin_z,
      max_distance;

    // auto ee_position = rm_->computeFK(request_common_msg.start_state.joint_state.position)
    //                      .translation();

    df_size_x = request_common_msg.workspace_parameters.max_corner.x
                - request_common_msg.workspace_parameters.min_corner.x;
    df_size_y = request_common_msg.workspace_parameters.max_corner.y
                - request_common_msg.workspace_parameters.min_corner.y;
    df_size_z = request_common_msg.workspace_parameters.max_corner.z
                - request_common_msg.workspace_parameters.min_corner.z;
    df_origin_x = -df_size_x / 2;
    df_origin_y = -df_size_y / 2;
    df_origin_z = 0.0;
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
    grid_ = new smpl::OccupancyGrid(df, ref_counted);  // will be deleted in cc_ destructor

    grid_->setReferenceFrame(planning_frame_);
    SV_SHOW_INFO(grid_->getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    if (verbose_) {
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
          grid_,
          grid_vec_,
          robot_description_,
          cc_conf,
          robot_config_.group_name,
          robot_config_.planning_joints
        )) {
        ROS_ERROR("Failed to initialize Collision Space");
        return false;
    }

    // TODO: remove (for debugging) ===========================
    // std::vector<std::string> const overlapping_links =
    //   {"bellows_link", "bellows_link2", "torso_fixed_link", "torso_lift_link"};
    // ========================================================

    // Load allowed collision pairs
    smpl::collision::AllowedCollisionMatrix acm;
    auto const link_names = scene_common_msg.allowed_collision_matrix.entry_names;
    auto const collision_values = scene_common_msg.allowed_collision_matrix.entry_values;
    for (unsigned int i = 0; i < link_names.size(); i++) {
        // ========================================================
        // auto link1_can_overlap = std::find(
        //                               overlapping_links.begin(),
        //                               overlapping_links.end(),
        //                               link_names[i]
        //                             )
        //                             != overlapping_links.end();
        // ========================================================

        for (unsigned int j = i + 1; j < link_names.size(); j++) {
            acm.setEntry(link_names[i], link_names[j], collision_values[i].enabled[j]);

            // ========================================================
            // auto link2_can_overlap = std::find(
            //                               overlapping_links.begin(),
            //                               overlapping_links.end(),
            //                               link_names[j]
            //                             )
            //                             != overlapping_links.end();
            //
            // // If both links are allowed to overlap, disable their collision checking
            // if (link1_can_overlap && link2_can_overlap) {
            //     acm.setEntry(link_names[i], link_names[j], true);
            // }
            // ========================================================
        }
    }

    cc_.setAllowedCollisionMatrix(acm);

    /////////////////
    // Setup Scene //
    /////////////////

    if (verbose_) {
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

    // SV_SHOW_INFO(grid_->getDistanceFieldVisualization(0.2));

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
    planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, grid_vec_);
    if (!planner_interface_->init(planner_params_)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return false;
    }

    if (visualize_) {
        VisualizeCollisionWorld();
    }

    return true;
}

bool Planner::planForProblemIdx(int problem_index, bool reverse) {
    // Read the specified problem's parameters
    moveit_msgs::MotionPlanRequest request_msg;
    moveit_msgs::PlanningScene scene_msg;
    if (!loadYamlToMsg(problems_dir_, problem_index, request_msg)) {
        ROS_ERROR(
          "Could not read problem %d's request message. Are the contents of %s properly formatted?",
          problem_index,
          problems_dir_.c_str()
        );
    }
    if (!loadYamlToMsg(problems_dir_, problem_index, scene_msg)) {
        ROS_ERROR(
          "Could not read problem %d's scene message. Are the contents of %s properly formatted?",
          problem_index,
          problems_dir_.c_str()
        );
    }

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(
          request_msg.start_state,
          scene_msg.fixed_frame_transforms
        )) {
        ROS_ERROR("Failed to set planning and collision reference state");
        return false;
    }

    // Remove all previous collision objects from scene and add those of the current problem
    if (!prepareCSSceneCollisionObjects(scene_msg.world.collision_objects)) {
        ROS_ERROR(
          "Failed to prepare collision space scene with the problem's collision objects"
        );
        return false;
    }

    // Export all occupied voxel centers (for visualization)
    if (!exportOccupiedVoxels()) {
        ROS_ERROR("Failed to export the scene's occupied voxel center coordinates");
        return false;
    }

    // Set planning algorithm id
    if (goal_type_ == "pose") {
        request_msg.planner_id = planning_algorithm_ + ".bfs.manip";
    } else if (goal_type_ == "joints") {
        request_msg.planner_id = planning_algorithm_ + ".joint_distance_weighted.manip";
    } else {
        ROS_ERROR("Goal type not identified!");
        return false;
    }

    // Swap start and goal in case of the reverse problem
    if (reverse) {
        swapStartAndGoal(request_msg.start_state, request_msg.goal_constraints[0]);
    }

    // Set up planner interface
    // planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, grid_vec_);
    // if (!planner_interface_->init(planner_params_)) {
    //     ROS_ERROR("Failed to initialize Planner Interface");
    //     return false;
    // }

    // plan
    if (verbose_) {
        ROS_INFO("Calling solve...");
    }

    moveit_msgs::MotionPlanResponse res;
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = request_msg.start_state;
    // request_msg.allowed_planning_time = 3;  // TODO: remove (for debugging)

    // TODO: remove (for debugging) ===========================
    // planner_interface_->checkStart(planning_scene, request_msg, res);
    // VisualizeCollisionWorld();
    // return true;
    // ========================================================

    auto plan_found = planner_interface_->solve(planning_scene, request_msg, res);

    auto planning_stats = planner_interface_->getPlannerStats();
    if (verbose_) {
        ROS_INFO("Planning statistics");
        for (auto & entry : planning_stats) {
            ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
        }
    }

    // Write planning stats to stats file
    stats_file_ << problem_index << separator_;
    for (auto iter = planning_stats.begin(); iter != planning_stats.end(); iter++) {
        stats_file_ << iter->second;
        if (std::next(iter) != planning_stats.end()) {
            stats_file_ << separator_;
        }
    }
    stats_file_ << std::endl;

    if (visualize_) {
        VisualizeCollisionWorld();
        VisualizePath(res.trajectory);
    }

    if ((!plan_found) || (res.trajectory.joint_trajectory.points.size() == 0)) {
        if (verbose_) {
            ROS_ERROR("Failed to plan");
        }
        return false;
    }

    return true;
}

bool Planner::ProcessCollisionObjects(
  std::vector<moveit_msgs::CollisionObject> & objects,
  moveit_msgs::CollisionObject::_operation_type operation
) {
    for (auto & object : objects) {
        object.header.frame_id = planning_frame_;
        object.operation = operation;

        for (int tidx = 0; tidx < num_threads_; ++tidx) {
            if (!cs_scene_.ProcessCollisionObjectMsg(tidx, object)) {
                if (verbose_) {
                    ROS_INFO(
                      "Failed to process collision object %s for thread %d",
                      object.id.c_str(),
                      tidx
                    );
                    return false;
                }
            } else {
                if (verbose_) {
                    ROS_INFO(
                      "Successfully processed collision object %s for thread %d",
                      object.id.c_str(),
                      tidx
                    );
                }
            }
        }
    }

    if (verbose_) {
        ROS_INFO("All collision objects successfully processed!");
    }

    return true;
}

bool Planner::prepareCSSceneCollisionObjects(std::vector<moveit_msgs::CollisionObject> & objects
) {
    // Remove any previous collision objects from scene
    if (!ProcessCollisionObjects(collision_objects_, moveit_msgs::CollisionObject::REMOVE)) {
        ROS_ERROR("Failed to remove all previous collision objects from scene");
        return false;
    }

    // Add the current planning problem's collision objects to scene
    if (!ProcessCollisionObjects(objects, moveit_msgs::CollisionObject::ADD)) {
        ROS_ERROR("Failed to add all collision objects to planning scene");
        return false;
    } else {
        collision_objects_ = objects;
    }

    if (visualize_) {
        VisualizeCollisionWorld();
    }

    return true;
}

bool Planner::exportOccupiedVoxels() {
    std::ofstream outFile("occupied_voxels.csv");  // ~/.ros/occupied_voxels.csv
    std::vector<Eigen::Vector3d> occupied_voxels_vec;

    std::string const separator = ",";

    outFile << "x" << separator << "y" << separator << "z\n";  // add header

    for (auto & grid : grid_vec_) {
        grid->getOccupiedVoxels(occupied_voxels_vec);
    }

    for (auto & voxel : occupied_voxels_vec) {
        outFile << std::to_string(voxel[0]) << separator << std::to_string(voxel[1])
                << separator << std::to_string(voxel[2]) << "\n";
    }

    outFile.close();
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

void Planner::VisualizePath(moveit_msgs::RobotTrajectory trajectory) {
    if (verbose_) {
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
    }
}


// Private //

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
        ROS_ERROR("Failed to read 'group_name' from the motion plan request configuration file"
        );
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

    if (verbose_) {
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
        if (verbose_) {
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
        ROS_ERROR("Failed to retrieve param 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", planner_config_.mprim_filename)) {
        ROS_ERROR("Failed to retrieve param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", planner_config_.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to retrieve param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", planner_config_.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to retrieve param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", planner_config_.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to retrieve param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", planner_config_.use_short_dist_mprims)) {
        ROS_ERROR("Failed to retrieve param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", planner_config_.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to retrieve param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", planner_config_.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to retrieve param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", planner_config_.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to retrieve param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", planner_config_.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to retrieve param 'use_xyz_snap_mprim' from the param server");
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

bool Planner::swapStartAndGoal(
  moveit_msgs::RobotState & start_state,
  moveit_msgs::Constraints & goal_state
) {
    // Iterate over the goal joint values
    for (auto & joint_constraint : goal_state.joint_constraints) {
        // Find the corresponding index for that joint in the start state array
        auto start_joint_index = distance(
          start_state.joint_state.name.begin(),
          std::find(
            start_state.joint_state.name.begin(),
            start_state.joint_state.name.end(),
            joint_constraint.joint_name
          )
        );

        // Swap the values between start and goal positions for this joint
        std::swap(
          start_state.joint_state.position[start_joint_index],
          joint_constraint.position
        );
    }
    return true;
}
