#ifndef COLLISION_SPACE_SCENE_MULTITHREAD_HPP
#define COLLISION_SPACE_SCENE_MULTITHREAD_HPP

#include <memory>
#include <vector>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <sbpl_collision_checking/collision_space_multithread.h>
#include <sbpl_collision_checking/shapes.h>

class CollisionSpaceSceneMultithread
{
public:

    void SetCollisionSpace(smpl::collision::CollisionSpaceMultithread* cspace);

    bool SetRobotState(int thread_idx, const moveit_msgs::RobotState& robot_state);

    bool ProcessCollisionObjectMsg(int thread_idx, const moveit_msgs::CollisionObject& object);
    bool AddCollisionObjectMsg(int thread_idx, const moveit_msgs::CollisionObject& object);
    bool RemoveCollisionObjectMsg(int thread_idx, const moveit_msgs::CollisionObject& object);
    bool AppendCollisionObjectMsg(int thread_idx, const moveit_msgs::CollisionObject& object);
    bool MoveCollisionObjectMsg(int thread_idx, const moveit_msgs::CollisionObject& object);

    // bool ProcessOctomapMsg(const octomap_msgs::OctomapWithPose& octomap);

    bool ProcessAttachedCollisionObject(
        int thread_idx,
        const moveit_msgs::AttachedCollisionObject& obj);

    // bool UpdatePlanningSceneWorld(
    //     const moveit_msgs::PlanningSceneWorld& world,
    //     bool is_diff);

    // bool UpdatePlanningScene(int thread_idx, const moveit_msgs::PlanningScene& scene);
    auto FindCollisionObject(const std::string& id) const -> smpl::collision::CollisionObject*;

    bool CheckCollisionObjectSanity(const moveit_msgs::CollisionObject& object) const;
    // bool CheckInsertOctomap(const octomap_msgs::OctomapWithPose& octomap) const;

private:

    std::vector<std::unique_ptr<octomap::OcTree>> m_octrees;
    std::vector<std::unique_ptr<smpl::collision::CollisionShape>> m_collision_shapes;
    std::vector<std::unique_ptr<smpl::collision::CollisionObject>> m_collision_objects;
    smpl::collision::CollisionSpaceMultithread *m_cspace = nullptr;

};

#endif
