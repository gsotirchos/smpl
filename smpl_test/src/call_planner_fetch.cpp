//////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the
// following conditions are met:
//
//     1. Redistributions of source code must retain the above
//        copyright notice this list of conditions and the
//        following disclaimer.
//     2. Redistributions in binary form must reproduce the above
//        copyright notice, this list of conditions and the
//        following disclaimer in the documentation and/or other
//        materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names
//        of its contributors may be used to endorse or promote
//        products derived from this software without specific
//        prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
// CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

// standard includes
#include <ros/service.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

// system includes
// #include <eigen_conversions/eigen_msg.h>
// #include <kdl_conversions/kdl_msg.h>
// #include <leatherman/print.h>
// #include <leatherman/utils.h>
// #include <moveit_msgs/GetMotionPlan.h>
// #include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
// #include <sbpl_kdl_robot_model/kdl_robot_model.h>
// #include <smpl/angles.h>
// #include <smpl/debug/visualizer_ros.h>
// #include <smpl/distance_map/edge_euclid_distance_map.h>
// #include <smpl/distance_map/euclid_distance_map.h>
// #include <smpl/ros/planner_interface.h>
// #include <smpl/ros/propagation_distance_field.h>
// #include <visualization_msgs/MarkerArray.h>

// #include "collision_space_scene_multithread.h"
// #include "franka_allowed_collision_pairs.h"
// #include "pr2_allowed_collision_pairs.h"
#include "planner.h"

// #include <sym_plan_msgs/ProcessAttachedCollisionObject.h>
#include <sym_plan_msgs/ProcessCollisionObject.h>
#include <sym_plan_msgs/RequestPlan.h>

/*
void FillGoalConstraint(
  std::vector<double> const & pose,
  std::string frame_id,
  moveit_msgs::Constraints & goals
) {
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type =
shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

    //    goals.position_constraints[0].position.x = pose[0];
    //    goals.position_constraints[0].position.y = pose[1];
    //    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
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

auto GetCollisionCubes(
  std::vector<std::vector<double>> & objects,
  std::vector<std::string> & object_ids,
  std::string const & frame_id
) -> std::vector<moveit_msgs::CollisionObject> {
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3, 0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(std::string const & filename, std::string const & frame_id)
  -> std::vector<moveit_msgs::CollisionObject> {
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double>> objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE * fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg, "%s", sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file", num_obs);

    // get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i = 0; i < num_obs; ++i) {
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
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(ros::NodeHandle & nh, moveit_msgs::RobotState & state) {
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                } else {
                    ROS_DEBUG(
                      "Doubles in the yaml file have to contain decimal points. (Convert '0' to
'0.0')"
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
                for (int i = 0; i < xlist.size(); ++i) {
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

    ROS_INFO(
      "Read initial state containing %zu joints and %zu multi-dof joints",
      state.joint_state.name.size(),
      state.multi_dof_joint_state.joint_names.size()
    );
    return true;
}

struct RobotModelConfig {
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(ros::NodeHandle const & nh, RobotModelConfig & config) {
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

bool ReadPlannerConfig(ros::NodeHandle const & nh, PlannerConfig & config) {
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

auto SetupRobotModel(std::string const & urdf, RobotModelConfig const & config, int
num_threads)
  -> std::unique_ptr<smpl::KDLRobotModel> {
    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR(
          "Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param
server"
        );
        return NULL;
    }

    ROS_INFO("Construct Generic KDL Robot Model");
    std::unique_ptr<smpl::KDLRobotModel> rm(new smpl::KDLRobotModel);

    if (!rm->init(urdf, config.kinematics_frame, config.chain_tip_link, num_threads)) {
        ROS_ERROR("Failed to initialize robot model.");
        return NULL;
    }

    return std::move(rm);
}
*/


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle const ph("~");

    //////////////
    // Planning //
    //////////////

    // Let server(s) set up
    ros::service::waitForService(symplan::Planner::planner_service_name, ros::Duration(1.0));
    ros::Duration(10.0).sleep();

    //== TODO: replace this with loading from yaml files ==================================
    sym_plan_msgs::RequestPlan planner_srv;

    planner_srv.request.goal_type = "pose";

    planner_srv.request.start_state.joint_state.position = std::vector<double>(7, 0);

    ph.param("goal/x", planner_srv.request.goal[0], 0.4);
    ph.param("goal/y", planner_srv.request.goal[1], -0.2);
    ph.param("goal/z", planner_srv.request.goal[2], 0.36);
    ph.param("goal/roll", planner_srv.request.goal[3], 0.0);
    ph.param("goal/pitch", planner_srv.request.goal[4], 0.0);
    ph.param("goal/yaw", planner_srv.request.goal[5], 0.0);
    //=====================================================================================

    {  //== TODO: process loaded collision objects ==============
        /*
        std::ofstream outFile("../occupied_voxels.csv");
        std::vector<Eigen::Vector3d> occupied_voxels_vec;

        std::string const separator = ",";

        outFile << "x" << separator << "y" << separator << "z\n";  // add header

        for (auto & grid : grid_vec) {
            grid->getOccupiedVoxels(occupied_voxels_vec);
        }

        for (auto & voxel : occupied_voxels_vec) {
            outFile << std::to_string(voxel[0]) << separator << std::to_string(voxel[1])
                    << separator << std::to_string(voxel[2]) << "\n";
        }

        outFile.close();
        */
    }  //========================================================

    // plan
    auto planner_service_client = nh.serviceClient<sym_plan_msgs::RequestPlan>(
      symplan::Planner::planner_service_name
    );

    if (!planner_service_client.call(planner_srv)) {
        ROS_ERROR("Failed when calling the planner service.");
    }

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    // TODO: these are printed after requesting a plan(?)
    // auto planning_stats = planner.getPlannerStats();
    //
    // ROS_INFO("Planning statistics");
    // for (auto & entry : planning_stats) {
    //    ROS_INFO("    %s: %0.3f", entry.first.c_str(),
    //    entry.second);
    //}

    // TODO: maybe needs Planner::VisualizeCollisionWorld()
    // ROS_INFO("Animate path");
    //
    // auto markers = cc.getCollisionWorldVisualization(0);
    // auto occupied_voxels =
    // cc.getOccupiedVoxelsVisualization();
    // SV_SHOW_INFO(markers);
    // SV_SHOW_INFO(occupied_voxels);
    //
    // size_t pidx = 0;
    // while (ros::ok()) {
    //    auto & point =
    //    res.trajectory.joint_trajectory.points[pidx]; auto
    //    markers_robot = cc.getCollisionRobotVisualization(0,
    //    point.positions); for (auto & m : markers.markers) {
    //        m.ns = "path_animation";
    //    }
    //    SV_SHOW_INFO(markers_robot);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    //    pidx++;
    //    pidx %= res.trajectory.joint_trajectory.points.size();
    //}

    return 0;
}
