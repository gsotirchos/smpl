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

    bool verbose;
    if (!ph.getParam("verbose", verbose)) {
        ROS_ERROR("Failed to read 'verbose' from the param server");
        return 1;
    }

    bool repeat_animation;
    if (!ph.getParam("repeat_animation", repeat_animation)) {
        ROS_ERROR("Failed to read 'repeat_animation' from the param server");
        return 1;
    }

    // Get planning problems' info
    std::string problems_dir;
    if (!ph.getParam("planning_problems_directory", problems_dir)) {
        ROS_ERROR("Failed to read 'planning_problems_directory' from the param server");
        return 1;
    }

    int problem_index_start;
    if (!ph.getParam("planning_problem_index_start", problem_index_start)) {
        ROS_ERROR("Failed to read 'planning_problem_index_start' from the param server");
        return 1;
    }

    int problem_index_end;
    if (!ph.getParam("planning_problem_index_end", problem_index_end)) {
        ROS_ERROR("Failed to read 'planning_problem_index_end' from the param server");
        return 1;
    }

    // Wait for an RViz instance
    while (!rvizIsRunning()) {
        ROS_INFO("Waiting for RViz instance to start before instantiating a planner...");
        ros::Duration(0.5).sleep();
    }

    Planner planner(nh, ph, verbose, repeat_animation);

    if (!planner.loadProblemCommonParams(problems_dir)) {
        ROS_ERROR(
          "Failed to load problem parameters from the specified directory: %s.",
          problems_dir.c_str()
        );
        return 1;
    }

    for (int i = problem_index_start; i <= problem_index_end; i++) {
        if (!planner.planForProblem(i)) {
            ROS_ERROR("Failed to plan for problem no. %d.", i);
            return 1;
        }

        if (verbose) {
            ROS_ERROR("Planning successful for problem no. %d.", i);
        }
    }

    ros::spin();

    return 0;
}
