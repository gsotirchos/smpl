// #include <common/Utilities.hpp>
#include <iostream>
#include <moveit_msgs/JointConstraint.h>
#include <thread>
#include <vector>

#include "franka_allowed_collision_pairs.h"
#include "pr2_allowed_collision_pairs.h"

#include "planner.h"

constexpr bool VERBOSE = true;
// #define print(x) (VERBOSE) ? ROS_INFO(x)


using namespace symplan;


// Public

Planner::Planner(
  std::string const & robot,
  ros::NodeHandle const & nh,
  ros::NodeHandle const & ph
) :
  robot_(robot),
  nh_(nh),
  ph_(ph) {
    if (VERBOSE) {
        ROS_INFO("Initialize visualizer");
    }
    smpl::VisualizerROS * visualizer_ = new smpl::VisualizerROS(nh_, 100);
    smpl::viz::set_visualizer(visualizer_);

    attached_co_id_.clear();
    cfg_ = std::unordered_map<std::string, double>();
}

Planner::~Planner() {
    delete visualizer_;
}

bool Planner::Init() {
    // TODO
    num_threads_ = (cfg_["algorithm"] == 0) ? 1 : static_cast<int>(cfg_["num_threads"]);

    /////////////////
    // Robot Model //
    /////////////////

    if (VERBOSE) {
        ROS_INFO("Load common parameters");
    }

    // Robot description required to initialize collision checker and robot model...
    auto robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh_.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return false;
    }

    if (!nh_.getParam(robot_description_param, robot_description_)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return false;
    }

    if (!readRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config_)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return false;
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

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    if (VERBOSE) {
        ROS_INFO("Initialize Occupancy Grid");
    }

    double df_size_x, df_size_y, df_size_z, df_res, df_origin_x, df_origin_y, df_origin_z,
      max_distance;

    // TODO: check whether these have to be hardcoded here
    if (robot_ == "pr2") {
        df_size_x = 1.5;
        df_size_y = 2.0;
        df_size_z = 2.0;
        df_res = 0.02;
        df_origin_x = -0.5;
        df_origin_y = -1.0;
        df_origin_z = 0;
        max_distance = 1.8;
    } else if (robot_ == "panda") {
        df_size_x = 1.5;
        df_size_y = 1.5;
        df_size_z = 1.0;
        df_res = 0.02;
        df_origin_x = -0.3;
        df_origin_y = -0.75;
        df_origin_z = 0.4;
        max_distance = 1.8;
    } else {
        ROS_ERROR("Robot collision model not recognized");
    }

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

    if (robot_ == "pr2") {
        if (robot_config_.group_name == "right_arm") {
            gripper_links_.push_back("r_gripper_l_finger_link");
            gripper_links_.push_back("r_gripper_r_finger_link");
            gripper_links_.push_back("r_gripper_l_finger_tip_link");
            gripper_links_.push_back("r_gripper_r_finger_tip_link");
            gripper_links_.push_back("r_gripper_palm_link");
        } else if (robot_config_.group_name == "left_arm") {
            gripper_links_.push_back("l_gripper_l_finger_link");
            gripper_links_.push_back("l_gripper_r_finger_link");
            gripper_links_.push_back("l_gripper_l_finger_tip_link");
            gripper_links_.push_back("l_gripper_r_finger_tip_link");
            gripper_links_.push_back("l_gripper_palm_link");
        } else {
            throw std::runtime_error("Arm not indetified in planner");
        }

        smpl::collision::AllowedCollisionMatrix acm;
        for (auto & pair : PR2AllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }

        cc_.setAllowedCollisionMatrix(acm);
    } else if (robot_ == "panda") {
        gripper_links_.push_back("panda_hand");
        gripper_links_.push_back("panda_leftfinger");
        gripper_links_.push_back("panda_rightfinger");

        smpl::collision::AllowedCollisionMatrix acm;
        for (auto & pair : FrankaAllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }

        acm.setEntry("panda_link0", "table_2", true);

        cc_.setAllowedCollisionMatrix(acm);
    } else {
        ROS_ERROR("Robot collision model not recognized");
    }

    // This does the same thing as turning the frames through config
    // TODO: ^ ??? and do I need this here?
    cc_.setWorldToModelTransform(0, Eigen::Affine3d::Identity());

    /////////////////
    // Setup Scene //
    /////////////////

    if (VERBOSE) {
        ROS_INFO("Initialize scene");
    }

    cs_scene_.SetCollisionSpace(&cc_);

    // TODO: I don't need this; we use object definitions
    // std::string object_filename;
    // ph_.param<std::string>("object_filename", object_filename, "");
    //
    // //Read in collision objects from file and add to the cs_scene_...
    // if (!object_filename.empty()) {
    //     auto objects = getCollisionObjects(object_filename,
    //     planning_frame_); for (auto& object : objects) {
    //         cs_scene_.ProcessCollisionObjectMsg(object);
    //     }
    // }

    if (!setupRobotModel(robot_description_, robot_config_)) {
        ROS_ERROR("Failed to set up Robot Model");
        return false;
    }

    // Initialize start state
    if (!readInitialConfiguration(ph_, start_state_)) {
        ROS_ERROR("Failed to get initial configuration.");
        return false;
    }

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(start_state_)) {
        ROS_ERROR("Failed to set planning and collision reference state.");
        return false;
    }

    // std::vector<double> us = {0, -0.78539816, 0, -2.35619449, 0, 1.57079633, 0.78539816};
    // cc_.updateState(us);

    // SV_SHOW_INFO(grid_.getDistanceFieldVisualization(0.2));

    ///////////////////
    // Planner Setup //
    ///////////////////

    if (!readPlannerConfig(ros::NodeHandle("~planning"), planning_config_)) {
        ROS_ERROR("Failed to read planner config");
        return false;
    }

    planner_params_.addParam("num_threads", num_threads_);
    planner_params_.addParam("discretization", planning_config_.discretization);
    planner_params_.addParam("mprim_filename", planning_config_.mprim_filename);
    planner_params_.addParam("use_xyz_snap_mprim", planning_config_.use_xyz_snap_mprim);
    planner_params_.addParam("use_rpy_snap_mprim", planning_config_.use_rpy_snap_mprim);
    planner_params_.addParam("use_xyzrpy_snap_mprim", planning_config_.use_xyzrpy_snap_mprim);
    planner_params_.addParam("use_short_dist_mprims", planning_config_.use_short_dist_mprims);
    planner_params_.addParam("xyz_snap_dist_thresh", planning_config_.xyz_snap_dist_thresh);
    planner_params_.addParam("rpy_snap_dist_thresh", planning_config_.rpy_snap_dist_thresh);
    planner_params_.addParam(
      "xyzrpy_snap_dist_thresh",
      planning_config_.xyzrpy_snap_dist_thresh
    );
    planner_params_.addParam(
      "short_dist_mprims_thresh",
      planning_config_.short_dist_mprims_thresh
    );
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

    //== TODO: This too exists in the planning callback; Do I need it here too? ==========
    // Set up planner interface
    planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, grid_vec_);
    if (!planner_interface_->init(planner_params_)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return false;
    }
    //====================================================================================

    VisualizeCollisionWorld();

    // Advertise service servers
    planner_service_server_ = nh_.advertiseService(
      planner_service_name,
      &Planner::RequestPlanCallback,
      this
    );
    collision_object_service_server_ = nh_.advertiseService(
      collision_object_service_name,
      &Planner::ProcessCollisionObjectCallback,
      this
    );
    request_ik_service_server_ = nh_.advertiseService(
      request_ik_service_name,
      &Planner::requestIKCallback,
      this
    );

    return true;
}

bool Planner::RequestPlanCallback(
  sym_plan_msgs::RequestPlan::Request & request,
  sym_plan_msgs::RequestPlan::Response & response
) {
    if (request.goal_type.empty()) {
        request.goal_type = "pose";
    }

    // Set start state
    if (!setStartState(request.start_state.joint_state.position.data())) {
        ROS_ERROR("Failed to set start state.");
        return false;
    }

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(start_state_)) {
        ROS_ERROR("Failed to set planning and collision reference state.");
        response.success = false;
        return true;
    }

    // Call Planner
    std::vector<double> const goal{request.goal.begin(), request.goal.end()};

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    if (cfg_.find("allowed_planning_time") != cfg_.end()) {
        req.allowed_planning_time = cfg_["allowed_planning_time"];
    } else {
        ph_.param("allowed_planning_time", req.allowed_planning_time, 10.0);
    }

    req.goal_constraints.resize(1);
    fillGoalConstraint(goal, planning_frame_, req.goal_constraints[0], request.goal_type);
    req.group_name = robot_config_.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;

    std::string const algo = (cfg_["algorithm"] == 0) ? "arastar" : "epase";
    if (request.goal_type == "pose") {
        req.planner_id = algo + ".bfs.manip";
    } else if (request.goal_type == "joints") {
        req.planner_id = algo + ".joint_distance.manip";
    } else {
        throw std::runtime_error("Goal type not identified!");
    }

    req.start_state = start_state_;

    //== TODO: This exists in Init() too; Do I need it here? ====================
    // Set up planner interface
    planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, &grid_);

    if (!planner_interface_->init(planner_params_)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        response.success = false;
        return true;
    }
    //===========================================================================

    // plan
    if (VERBOSE) {
        ROS_INFO("Calling solve...");
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = start_state_;
    auto plan_found = planner_interface_->solve(planning_scene, req, res);
    if ((!plan_found) || (res.trajectory.joint_trajectory.points.size() == 0)) {
        ROS_ERROR("Failed to plan.");
        response.success = false;
        response.cost = -1;
        return true;
    }

    auto planning_stats = planner_interface_->getPlannerStats();
    response.cost = planning_stats["solution cost"];
    response.time = planning_stats["time"];
    response.state_expansions = planning_stats["state_expansions"];
    response.edge_expansions = planning_stats["edge_expansions"];

    if (VERBOSE) {
        ROS_INFO("Planning statistics");
        for (auto & entry : planning_stats) {
            ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
        }
    }

    // Send service reponse
    response.success = true;
    response.plan = res;

    VisualizePath(res.trajectory);

    return true;
}

bool Planner::ProcessCollisionObjectCallback(
  sym_plan_msgs::ProcessCollisionObject::Request & request,
  sym_plan_msgs::ProcessCollisionObject::Response & response
) {
    if (VERBOSE) {
        std::cout << "-------------- " << __FUNCTION__ << " START ---------------" << '\n';
    }
    bool succ = true;
    for (auto & object : request.objects) {
        if (VERBOSE) {
            std::cout << "Processing collision object: " << std::string(object.id)
                      << " | operation: " << object.operation << '\n';
        }

        // If object is attached, detach it and add it to the
        // collision scene
        if ((object.id == attached_co_id_) && ((object.operation == moveit_msgs::CollisionObject::ADD) || (object.operation == moveit_msgs::CollisionObject::MOVE)))
        {
            moveit_msgs::AttachedCollisionObject detach_object;
            detach_object.object = object;
            detach_object.link_name = gripper_links_[0];
            detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

            for (int tidx = 0; tidx < num_threads_; ++tidx) {
                if (!cs_scene_.ProcessAttachedCollisionObject(tidx, detach_object)) {
                    succ = false;
                    if (VERBOSE) {
                        std::cout << "Could not detach collision object "
                                  << std::string(detach_object.object.id) << '\n';
                    }
                }
            }

            smpl::collision::AllowedCollisionMatrix acm_detached;
            if (VERBOSE) {
                std::cout << "Disabling collision with: " << attached_co_id_ << '\n';
            }
            for (auto & gripper_link : gripper_links_) {
                acm_detached.setEntry(attached_co_id_, gripper_link, false);
            }
            cc_.updateAllowedCollisionMatrix(acm_detached);
            attached_co_id_.clear();
        }

        if (object.operation == moveit_msgs::CollisionObject::MOVE) {
            if (!cs_scene_.FindCollisionObject(object.id)) {
                object.operation = moveit_msgs::CollisionObject::ADD;
            }
        }

        object.header.frame_id = planning_frame_;
        for (int tidx = 0; tidx < num_threads_; ++tidx) {
            if (!cs_scene_.ProcessCollisionObjectMsg(tidx, object)) {
                succ = false;
                if (VERBOSE) {
                    std::cout << "Could not process : " << object.operation
                              << " collision object " << std::string(object.id)
                              << " for thread " << tidx << '\n';
                }
            } else {
                if (VERBOSE) {
                    std::cout << "Successfully " << object.operation << " collision object "
                              << std::string(object.id) << " for thread " << tidx << '\n';
                }
            }
        }
    }

    VisualizeCollisionWorld();

    if (succ) {
        if (VERBOSE) {
            std::cout << "All collision objects successfully processed!" << '\n';
        }
    }

    response.success = succ;
    if (VERBOSE) {
        std::cout << "-------------- " << __FUNCTION__ << " END ---------------" << '\n';
    }

    return true;
}

/*
bool Planner::ProcessAttachedCollisionObjectCallback(
  sym_plan_msgs::ProcessAttachedCollisionObject::Request & request,
  sym_plan_msgs::ProcessAttachedCollisionObject::Response & response
) {
    if (VERBOSE) {
        std::cout << "-------------- " << __FUNCTION__ << " START ---------------" << '\n';
    }

    bool succ = true;
    for (auto & attached_object : request.attached_objects) {
        if (attached_object.object.operation == moveit_msgs::CollisionObject::ADD) {
            if (attached_co_id_ != attached_object.object.id) {
                smpl::collision::AllowedCollisionMatrix acm_detached;
                if (VERBOSE) {
                    std::cout << "Disabling collision with: " << attached_co_id_ << '\n';
                }
                for (auto & gripper_link : gripper_links_) {
                    acm_detached.setEntry(attached_co_id_, gripper_link, false);
                }
                cc_.updateAllowedCollisionMatrix(acm_detached);

                // Detach existing attached object if new object
                // not same
                moveit_msgs::AttachedCollisionObject detach_object;
                detach_object.object.id = attached_co_id_;
                detach_object.link_name = gripper_links_[0];
                detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

                for (int tidx = 0; tidx < num_threads_; ++tidx) {
                    if (!cs_scene_.ProcessAttachedCollisionObject(tidx, detach_object)) {
                        succ = false;
                        if (VERBOSE) {
                            std::cout << "Could not detach collision object "
                                      << std::string(detach_object.object.id) << " thread "
                                      << tidx << '\n';
                        }
                    }

                    // Remove new object from collision scene
                    if (cs_scene_.FindCollisionObject(attached_object.object.id)) {
                        auto remove_object = attached_object.object;
                        remove_object.header.frame_id = planning_frame_;
                        remove_object.operation = moveit_msgs::CollisionObject::REMOVE;
                        cs_scene_.ProcessCollisionObjectMsg(tidx, remove_object);
                    }

                    // Attach new object
                    if (!cs_scene_.ProcessAttachedCollisionObject(tidx, attached_object)) {
                        succ = false;
                        if (VERBOSE) {
                            std::cout << "Could not attach collision object "
                                      << std::string(attached_object.object.id) << " thread "
                                      << tidx << '\n';
                        }

                    } else {
                        if (VERBOSE) {
                            std::cout << "Successfully attached collision object "
                                      << std::string(attached_object.object.id) << " thread "
                                      << tidx << '\n';
                        }
                    }
                }

                attached_co_id_ = attached_object.object.id;
                std::cout << "Enabling collision with: " << attached_co_id_ << '\n';
                smpl::collision::AllowedCollisionMatrix acm_attached;

                for (auto & gripper_link : gripper_links_) {
                    acm_attached.setEntry(attached_co_id_, gripper_link, true);
                }
                cc_.updateAllowedCollisionMatrix(acm_attached);
            }
        }

        // std::cin.get();


        // ROS_INFO("Attached Body Count %zu \n",
        // cc_.m_abcs->model()->attachedBodyCount());
        // ROS_INFO("Has Attached Body(%s): %s \n",
        // attached_co_id_.c_str(),
        // cc_.m_abcm->hasAttachedBody(attached_co_id_) ? "true"
        // : "false"); const int abidx =
        // cc_.m_abcm->attachedBodyIndex(attached_co_id_);
        // ROS_INFO("Attached Body Index: %d \n", abidx);
        // ROS_INFO("Attached Body Name(%d): %s \n", abidx,
        // cc_.m_abcm->attachedBodyName(abidx).c_str());
        // ROS_INFO("Attached Body Indices: %s",
        // std::to_string(cc_.m_abcm->attachedBodyIndices(attach_link)).c_str());

        // std::cout << "Attached Body Count: "<<
        // cc_.m_abcs->model()->attachedBodyCount() << '\n';
        // printf("Has Attached Body(%s): %s \n",
        // attached_co_id_.c_str(),
        // cc_.m_abcm->hasAttachedBody(attached_co_id_) ? "true"
        // : "false"); const int abidx =
        // cc_.m_abcm->attachedBodyIndex(attached_co_id_); std::cout
        // << "Attached Body Index: " << abidx << '\n';
        // printf("Attached Body Name(%d): %s \n", abidx,
        // cc_.m_abcm->attachedBodyName(abidx).c_str());
    }
    // std::cout << "Attached object: "
    // <<request.attached_objects[0].object.id << "| Press enter
    // to visulize\n"; getchar(); VisualizeCollisionWorld(); std::cout
    // << "Visualziation complete \n"; getchar();
    if (succ) {
        if (VERBOSE) {
            std::cout << "All attached collision objects successfully processed!" << '\n';
        }
    }
    response.success = succ;

    if (VERBOSE) {
        std::cout << "-------------- " << __FUNCTION__ << " END ---------------" << '\n';
    }

    return true;
}
*/

Eigen::Affine3d Planner::ComputeFK(std::vector<double> const & joints) {
    return rm_->computeFK(joints);
}

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

bool Planner::CheckStartState(
  std::vector<double> const & start_state_joints,
  std::vector<double> const & goal,
  std::string const & goal_type
) {
    // Set start state
    if (!setStartState(start_state_joints.data())) {
        ROS_ERROR("Failed to set start state.");
        return false;
    }

    // Set reference state for planning and collision
    if (!setPlanningAndCollisionReferenceState(start_state_)) {
        return false;
    }

    // planner_interface_ = std::make_shared<smpl::PlannerInterface>(rm_.get(), &cc_, &grid_);
    // if (!planner_interface_->init(planner_params_))
    // {
    //     ROS_ERROR("Failed to initialize Planner Interface");
    //     return false;
    // }

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    req.goal_constraints.resize(1);
    fillGoalConstraint(goal, planning_frame_, req.goal_constraints[0], goal_type);
    req.group_name = robot_config_.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;

    std::string const algo = (cfg_["algorithm"] == 0) ? "arastar" : "epase";

    req.planner_id = algo + ".bfs.manip";
    req.start_state = start_state_;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = start_state_;

    return planner_interface_->checkStart(planning_scene, req, res);
}

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


ros::NodeHandle & Planner::GetNh() {
    return nh_;
}

ros::NodeHandle & Planner::GetPh() {
    return ph_;
}

std::string Planner::GetRobot() {
    return robot_;
}

std::unordered_map<std::string, double> Planner::GetConfig() {
    return cfg_;
}


// Private

bool Planner::requestIKCallback(
  sym_plan_msgs::RequestIK::Request & request,
  sym_plan_msgs::RequestIK::Response & response
) {
    std::vector<double> const seed_joint_states(7, 0);
    std::vector<double> solution;
    Eigen::Transform<double, 3, Eigen::Affine> ee_pose;
    Eigen::AngleAxisd const rollAngle(request.orientation[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd const pitchAngle(request.orientation[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd const yawAngle(request.orientation[2], Eigen::Vector3d::UnitZ());
    // ee_pose = yawAngle * pitchAngle * rollAngle;
    // Eigen::AngleAxisd rollAngle(request.orientation[0],
    // Eigen::Vector3d::UnitZ()); Eigen::AngleAxisd
    // yawAngle(request.orientation[1],
    // Eigen::Vector3d::UnitY()); Eigen::AngleAxisd
    // pitchAngle(request.orientation[2],
    // Eigen::Vector3d::UnitX());

    ee_pose = rollAngle * yawAngle * pitchAngle;

    ee_pose.pretranslate(
      Eigen::Vector3d(request.position[0], request.position[1], request.position[2])
    );

    bool const success = rm_->computeIK(
      ee_pose,
      seed_joint_states,
      solution,
      smpl::ik_option::UNRESTRICTED
    );
    response.success = success;
    response.joints = solution;
    if (!response.success) {
        ROS_WARN("Failed IK !");
    }

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
        ROS_ERROR("Failed to initialize robot model.");
        return false;
    }

    // To handle smpl bug where IK does not work without one call to FK
    // std::vector<double> const joint_states =
    //  {0.45274857, 0.58038735, -0.11977164, -1.9721714, 0.13661714, 2.5460937, 1.040775};
    //
    // for (int tidx = 0; tidx < num_threads_; ++tidx) {
    //    auto fk_solution = rm_->computeFK(joint_states, tidx);
    //}

    // std::vector<double> solution;

    // bool success = rm_->computeIK(fk_solution, joint_states,
    // solution, smpl::ik_option::UNRESTRICTED); auto ea =
    // fk_solution.rotation().eulerAngles(0,1,2); std::cout <<
    // "Euler from quaternion in roll, pitch, yaw"<< '\n' <<
    // ea << '\n'; std::cout << "pose " <<
    // fk_solution.translation() << '\n';
    return true;
}

bool Planner::readPlannerConfig(ros::NodeHandle const & nh, PlannerConfig & config) {
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

bool Planner::readRobotModelConfig(ros::NodeHandle const & nh, RobotModelConfig & config) {
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

bool Planner::readInitialConfiguration(ros::NodeHandle & nh, moveit_msgs::RobotState & state) {
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (auto i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                } else {
                    ROS_DEBUG(
                      "Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')"
                    );
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    } else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto & multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (auto i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
                      (double)xlist[i]["yaw"],
                      (double)xlist[i]["pitch"],
                      (double)xlist[i]["roll"],
                      q
                    );

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    if (VERBOSE) {
        ROS_INFO(
          "Read initial state containing %zu joints and %zu multi-dof joints",
          state.joint_state.name.size(),
          state.multi_dof_joint_state.joint_names.size()
        );
    }

    return true;
}

bool Planner::setStartState(double const * state) {
    for (auto i = 0; i < start_state_.joint_state.name.size(); ++i) {
        if (cc_.robotCollisionModel()->name() == "pr2") {
            if (robot_config_.group_name == "right_arm") {
                if (start_state_.joint_state.name[i] == "r_shoulder_pan_joint") {
                    start_state_.joint_state.position[i] = state[0];
                } else if (start_state_.joint_state.name[i] == "r_shoulder_lift_joint") {
                    start_state_.joint_state.position[i] = state[1];
                } else if (start_state_.joint_state.name[i] == "r_upper_arm_roll_joint") {
                    start_state_.joint_state.position[i] = state[2];
                } else if (start_state_.joint_state.name[i] == "r_elbow_flex_joint") {
                    start_state_.joint_state.position[i] = state[3];
                } else if (start_state_.joint_state.name[i] == "r_forearm_roll_joint") {
                    start_state_.joint_state.position[i] = state[4];
                } else if (start_state_.joint_state.name[i] == "r_wrist_flex_joint") {
                    start_state_.joint_state.position[i] = state[5];
                } else if (start_state_.joint_state.name[i] == "r_wrist_roll_joint") {
                    start_state_.joint_state.position[i] = state[6];
                }
            } else if (robot_config_.group_name == "left_arm") {
                if (start_state_.joint_state.name[i] == "l_shoulder_pan_joint") {
                    start_state_.joint_state.position[i] = state[0];
                } else if (start_state_.joint_state.name[i] == "l_shoulder_lift_joint") {
                    start_state_.joint_state.position[i] = state[1];
                } else if (start_state_.joint_state.name[i] == "l_upper_arm_roll_joint") {
                    start_state_.joint_state.position[i] = state[2];
                } else if (start_state_.joint_state.name[i] == "l_elbow_flex_joint") {
                    start_state_.joint_state.position[i] = state[3];
                } else if (start_state_.joint_state.name[i] == "l_forearm_roll_joint") {
                    start_state_.joint_state.position[i] = state[4];
                } else if (start_state_.joint_state.name[i] == "l_wrist_flex_joint") {
                    start_state_.joint_state.position[i] = state[5];
                } else if (start_state_.joint_state.name[i] == "l_wrist_roll_joint") {
                    start_state_.joint_state.position[i] = state[6];
                }
            } else {
                ROS_ERROR("Arm not specified in planner");
                return false;
            }
        } else if (cc_.robotCollisionModel()->name() == "panda") {
            if (start_state_.joint_state.name[i] == "panda_joint1") {
                start_state_.joint_state.position[i] = state[0];
            } else if (start_state_.joint_state.name[i] == "panda_joint2") {
                start_state_.joint_state.position[i] = state[1];
            } else if (start_state_.joint_state.name[i] == "panda_joint3") {
                start_state_.joint_state.position[i] = state[2];
            } else if (start_state_.joint_state.name[i] == "panda_joint4") {
                start_state_.joint_state.position[i] = state[3];
            } else if (start_state_.joint_state.name[i] == "panda_joint5") {
                start_state_.joint_state.position[i] = state[4];
            } else if (start_state_.joint_state.name[i] == "panda_joint6") {
                start_state_.joint_state.position[i] = state[5];
            } else if (start_state_.joint_state.name[i] == "panda_joint7") {
                start_state_.joint_state.position[i] = state[6];
            } else if (start_state_.joint_state.name[i] == "panda_finger_joint1") {
                start_state_.joint_state.position[i] = state[7];
            } else if (start_state_.joint_state.name[i] == "panda_finger_joint2") {
                start_state_.joint_state.position[i] = state[8];
            } else if (start_state_.joint_state.name[i] == "map/trans_x") {
                start_state_.joint_state.position[i] = state[9];
            } else if (start_state_.joint_state.name[i] == "map/trans_y") {
                start_state_.joint_state.position[i] = state[10];
            } else if (start_state_.joint_state.name[i] == "map/trans_z") {
                start_state_.joint_state.position[i] = state[11];
            } else if (start_state_.joint_state.name[i] == "map/rot_x") {
                start_state_.joint_state.position[i] = state[12];
            } else if (start_state_.joint_state.name[i] == "map/rot_y") {
                start_state_.joint_state.position[i] = state[13];
            } else if (start_state_.joint_state.name[i] == "map/rot_z") {
                start_state_.joint_state.position[i] = state[14];
            } else if (start_state_.joint_state.name[i] == "map/rot_w") {
                start_state_.joint_state.position[i] = state[15];
            }
        }
    }

    return true;
}

bool Planner::setPlanningAndCollisionReferenceState(moveit_msgs::RobotState & state) {
    // Set reference state in the robot planning model...
    smpl::urdf::RobotState reference_state;
    InitRobotState(&reference_state, &rm_->m_robot_model);
    for (auto i = 0; i < state.joint_state.name.size(); ++i) {
        auto * var = GetVariable(&rm_->m_robot_model, &state.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN("Failed to do the thing");
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

    // Set reference state in the collision model...
    for (auto tidx = 0; tidx < num_threads_; ++tidx) {
        if (!cs_scene_.SetRobotState(tidx, state)) {
            ROS_ERROR("Failed to set start state on Collision Space Scene");
            return false;
        }

        // TODO: might have to modify this
        cc_.setWorldToModelTransform(tidx, Eigen::Affine3d::Identity());
    }

    return true;
}

void Planner::fillGoalConstraint(
  std::vector<double> const & pose,
  std::string const & frame_id,
  moveit_msgs::Constraints & goals,
  std::string const & goal_type
) {
    if (goal_type == "pose") {
        if (pose.size() < 6) {
            return;
        }

        goals.position_constraints.resize(1);
        goals.orientation_constraints.resize(1);
        goals.position_constraints[0].header.frame_id = frame_id;

        goals.position_constraints[0].constraint_region.primitives.resize(1);
        goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
        goals.position_constraints[0]
          .constraint_region.primitives[0]
          .type = shape_msgs::SolidPrimitive::BOX;
        goals.position_constraints[0]
          .constraint_region.primitive_poses[0]
          .position.x = pose[0];
        goals.position_constraints[0]
          .constraint_region.primitive_poses[0]
          .position.y = pose[1];
        goals.position_constraints[0]
          .constraint_region.primitive_poses[0]
          .position.z = pose[2];

        Eigen::Quaterniond q;
        smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
        tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

        geometry_msgs::Pose p;
        p.position = goals.position_constraints[0]
                       .constraint_region.primitive_poses[0]
                       .position;
        p.orientation = goals.orientation_constraints[0].orientation;

        if (VERBOSE) {
            leatherman::printPoseMsg(p, "Goal");
        }

        /// set tolerances
        goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(
          3,
          0.015
        );
        goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.01;
        goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.01;
        goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.01;

        if (VERBOSE) {
            ROS_INFO("Done packing the goal constraints message.");
        }
    } else if (goal_type == "joints") {
        goals.joint_constraints.resize(robot_config_.planning_joints.size());
        for (auto idx = 0; idx < robot_config_.planning_joints.size(); ++idx) {
            moveit_msgs::JointConstraint jc;
            jc.joint_name = robot_config_.planning_joints[idx];
            jc.position = pose[idx];
            jc.tolerance_above = 0.01;
            jc.tolerance_below = 0.01;
            jc.weight = 1;
            goals.joint_constraints[idx] = jc;
        }
    } else {
        throw std::runtime_error("Goal type not identified!");
    }
}

auto Planner::getCollisionCube(
  geometry_msgs::Pose const & pose,
  std::vector<double> & dims,
  std::string const & frame_id,
  std::string const & id
) -> moveit_msgs::CollisionObject {
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

std::vector<moveit_msgs::CollisionObject> Planner::getCollisionCubes(
  std::vector<std::vector<double>> & objects,
  std::vector<std::string> & object_ids,
  std::string const & frame_id
) {
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3, 0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        if (VERBOSE) {
            ROS_INFO("object id list is not same length as object list. exiting.");
        }
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(getCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

std::vector<moveit_msgs::CollisionObject>
Planner::getCollisionObjects(std::string const & filename, std::string const & frame_id) {
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double>> objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE * fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        if (VERBOSE) {
            ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        }
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg, "%s", sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = std::stoi(sTemp, nullptr);

    if (VERBOSE) {
        ROS_INFO("%i objects in file", num_obs);
    }

    // get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (auto i = 0; i < num_obs; ++i) {
        if (fscanf(fCfg, "%s", sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j = 0; j < 6; ++j) {
            if (fscanf(fCfg, "%s", sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = std::strtod(sTemp, nullptr);
            }
        }
    }

    return getCollisionCubes(objects, object_ids, frame_id);
}
