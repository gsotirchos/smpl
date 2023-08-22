// standard includes
#include <ros/service.h>
// #include <stdlib.h>
#include <string>
// #include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
// #include <sym_plan_msgs/ProcessAttachedCollisionObject.h>
#include <sym_plan_msgs/ProcessCollisionObject.h>
#include <sym_plan_msgs/RequestPlan.h>

#include "planner.h"


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle const ph("~");

    //////////////
    // Planning //
    //////////////

    // Let server(s) set up
    ros::service::waitForService(symplan::Planner::planner_service_name, ros::Duration(1.0));
    ros::Duration(5.0).sleep();

    //== TODO: replace this with loading from yaml files ==================================
    sym_plan_msgs::RequestPlan planner_srv;

    planner_srv.request.goal_type = "pose";

    planner_srv.request.start_state.joint_state
      .position = {0.0, 0.0, 0.0, -1.1356, 0.0, -1.05, 0.0};
    planner_srv.request.start_state.attached_collision_objects =
      std::vector<moveit_msgs::AttachedCollisionObject>{};

    planner_srv.request.goal = {0.4, -0.2, 0.36, 0.0, 0.0, 0.0};
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
