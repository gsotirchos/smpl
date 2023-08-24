// standard includes
// #include <stdlib.h>
#include <string>
// #include <thread>
#include <vector>

// system includes
#include <ros/ros.h>
#include <ros/service.h>
// #include <sym_plan_msgs/ProcessAttachedCollisionObject.h>
#include <sym_plan_msgs/ProcessCollisionObject.h>
#include <sym_plan_msgs/RequestPlan.h>

#include "planner.h"


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle const ph("~");

    //== TODO: replace this with loading from yaml files ==================================
    sym_plan_msgs::RequestPlan planner_srv;

    planner_srv.request.goal_type = "pose";

    planner_srv.request.start_state.joint_state
      .position = {0.0, 0.0, 0.0, -1.1356, 0.0, -1.05, 0.0};
    planner_srv.request.start_state
      .attached_collision_objects = std::vector<moveit_msgs::AttachedCollisionObject>{};

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

    // Let server(s) set up
    ros::service::waitForService(symplan::Planner::planner_service_name, ros::Duration(1.0));
    ros::Duration(1.0).sleep();

    // plan
    auto planner_service_client = nh.serviceClient<sym_plan_msgs::RequestPlan>(
      symplan::Planner::planner_service_name
    );

    if (!planner_service_client.call(planner_srv)) {
        ROS_ERROR("Failed when calling the planner service.");
    }

    return 0;
}
