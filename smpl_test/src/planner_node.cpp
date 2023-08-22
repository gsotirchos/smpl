#include "planner.h"

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle const nh;
    ros::NodeHandle const ph("~");

    auto const robot_name = "pr2";

    symplan::Planner planner(robot_name, nh, ph);

    if (!planner.Init()) {
        ROS_ERROR("Failed to initialize planner instance.");
    }

    ros::spin();

    return 0;
}
