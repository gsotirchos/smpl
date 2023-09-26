// standard includes
#include <string>

// system includes
#include <ros/ros.h>

#include "planner.h"

bool rvizIsRunning() {
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

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle const nh;
    ros::NodeHandle const ph("~");

    std::string problems_dir;
    if (!ph.getParam("planning_problems_directory", problems_dir)) {
        ROS_ERROR("Failed to read 'planning_problems_directory' from the param server");
        return 1;
    }

    int problem_index;
    if (!ph.getParam("planning_problem_index", problem_index)) {
        ROS_ERROR("Failed to read 'planning_problem_index' from the param server");
        return 1;
    }

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


    while (!rvizIsRunning()) {
        ROS_INFO("Waiting for RViz instance to start before instantiating a planner...");
        ros::Duration(0.5).sleep();
    }

    Planner planner(nh, ph);

    if (!planner.loadProblemCommonParams(problems_dir)) {
        ROS_ERROR(
          "Failed to load problem parameters from the specified directory: %s.",
          problems_dir.c_str()
        );
        return 1;
    }

    if (!planner.planForProblem(problem_index)) {
        ROS_ERROR("Failed to plan for problem no. %d.", problem_index);
        return 1;
    }

    ros::spin();

    return 0;
}
