// standard includes
#include <string>

// system includes
#include <ros/ros.h>

#include "planner.h"


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle const nh;
    ros::NodeHandle const ph("~");

    bool verbose;
    if (!ph.getParam("verbose", verbose)) {
        ROS_ERROR("Failed to read 'verbose' from the param server");
        return 1;
    }

    bool visualize;
    if (!ph.getParam("visualize", visualize)) {
        ROS_ERROR("Failed to read 'visualize' from the param server");
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


    Planner planner(nh, ph, verbose, visualize);

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

    return 0;
}
