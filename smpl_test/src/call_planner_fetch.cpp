// standard includes
#include <string>
#include <vector>

// system includes
#include <ros/ros.h>
#include <ros/service.h>
// #include <sym_plan_msgs/ProcessAttachedCollisionObject.h>
#include <sym_plan_msgs/ProcessCollisionObject.h>
#include <sym_plan_msgs/RequestPlan.h>

#include "planner.h"

bool readInitialConfiguration(ros::NodeHandle & nh, moveit_msgs::RobotState & state) {
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
        ROS_ERROR("initial_configuration/joint_state is not on the param server.");
        return false;
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
                ROS_ERROR("initial_configuration/multi_dof_joint_state array is empty");
                return false;
            }
        } else {
            ROS_ERROR("initial_configuration/multi_dof_joint_state is not an array.");
            return false;
        }
    }

    ROS_INFO(
      "Read initial state containing %zu joints and %zu multi-dof joints",
      state.joint_state.name.size(),
      state.multi_dof_joint_state.joint_names.size()
    );

    return true;
}

// TODO: this
bool readFinalConfiguration(ros::NodeHandle & nh, std::vector<double> goal) {
    // XmlRpc::XmlRpcValue xlist;
    // std::vector<std::string> value_names;
    // double position;
    //
    // if (nh.hasParam("goal")) {
    //     nh.getParam("goal", xlist);
    //
    //     //if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    //     //    ROS_WARN("'goal' is not an array.");
    //     //}
    //
    //     if (xlist.size() > 0) {
    //         for (auto i = 0; i < xlist.size(); ++i) {
    //             value_names.push_back(std::string(xlist[i]["name"]));
    //
    //             if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    //                 position = xlist[i]["position"];
    //             } else {
    //                 ROS_DEBUG(
    //                   "Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')"
    //                 );
    //                 if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    //                     position = double(xlist[i]["position"]);
    //                 }
    //             }
    //             goal.push_back(position);
    //         }
    //     }
    // } else {
    //     ROS_ERROR("'goal' is not on the param server.");
    //     return false;
    // }

    return true;
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    //== TODO: load these from yaml files instead =========================================
    sym_plan_msgs::RequestPlan planner_srv;

    planner_srv.request.goal_type = "pose";

    // start state
    if (!readInitialConfiguration(ph, planner_srv.request.start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }

    // attached objects
    planner_srv.request.start_state
      .attached_collision_objects = std::vector<moveit_msgs::AttachedCollisionObject>{};

    // goal
    if (!readFinalConfiguration(ph, planner_srv.request.goal)) {
        ROS_ERROR("Failed to get final configuration.");
        return 1;
    }
    planner_srv.request.goal = {0.6, 0.0, 0.63, 0.0, 3.14, 0.0};
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
    ros::service::waitForService(symplan::Planner::planner_service_name);
    ros::Duration(1.0).sleep();

    // Plan
    auto planner_service_client = nh.serviceClient<sym_plan_msgs::RequestPlan>(
      symplan::Planner::planner_service_name
    );

    if (!planner_service_client.call(planner_srv)) {
        ROS_ERROR("Failed when calling the planner service.");
    }

    return 0;
}
