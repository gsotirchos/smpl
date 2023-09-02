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
    ros::init(argc, argv, "planner");
    ros::NodeHandle const nh;
    ros::NodeHandle const ph("~");
    std::string const problems_dir = "~/problems/panda/bookshelf_small_panda/";

    while (!rvizIsRunning()) {
        ROS_INFO("Waiting for RViz to start before instantiating a planner...");
        ros::Duration(0.5).sleep();
    }

    symplan::Planner planner(nh, ph, problems_dir);

    if (!planner.Init()) {
        ROS_ERROR("Failed to initialize planner instance.");
    }

    ros::spin();

    return 0;
}
