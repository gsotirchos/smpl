#include <moveit/collision_detection/world.h> // Object, ObjectPtr, ObjectConstPtr
#include <geometric_shapes/shape_operations.h> // constructShapeFromMsg
#include <geometric_shapes/shapes.h> // Shape, ShapePtr, ShapeConstPtr
#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>
#include <smpl/stl/memory.h>

#include "collision_space_scene_multithread.h"

static const char* LOG = "collision_space_scene";

using namespace smpl::collision;

auto ConvertCollisionObjectToObjectMultithread(const moveit_msgs::CollisionObject& co)
    -> std::unique_ptr<const collision_detection::World::Object>
{
    auto o = smpl::make_unique<collision_detection::World::Object>(co.id);

    for (size_t pidx = 0; pidx < co.primitives.size(); ++pidx) {
        const shape_msgs::SolidPrimitive& prim = co.primitives[pidx];
        const geometry_msgs::Pose& pose = co.primitive_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(prim));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from primitive message");
            return nullptr;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    for (size_t midx = 0; midx < co.meshes.size(); ++midx) {
        const shape_msgs::Mesh& mesh = co.meshes[midx];
        const geometry_msgs::Pose& pose = co.mesh_poses[midx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(mesh));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from mesh message");
            return nullptr;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    for (size_t pidx = 0; pidx < co.planes.size(); ++pidx) {
        const shape_msgs::Plane& plane = co.planes[pidx];
        const geometry_msgs::Pose& pose = co.plane_poses[pidx];

        shapes::ShapeConstPtr sp(shapes::constructShapeFromMsg(plane));
        if (!sp) {
            ROS_ERROR("Failed to construct shape from plane message");
            return nullptr;
        }

        Eigen::Isometry3d transform;
        tf::poseMsgToEigen(pose, transform);

        o->shapes_.push_back(sp);
        o->shape_poses_.push_back(transform);
    }

    return std::move(o);
}

auto ConvertOctomapToObjectMultithread(const octomap_msgs::OctomapWithPose& octomap)
    -> std::unique_ptr<const collision_detection::World::Object>
{
    // convert binary octomap message to octree
    octomap::AbstractOcTree* abstract_tree =
            octomap_msgs::binaryMsgToMap(octomap.octomap);
    if (!abstract_tree) {
        ROS_WARN_NAMED(LOG, "Failed to convert binary msg data to octomap");
        return nullptr;
    }

    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
    if (!tree) {
        ROS_WARN_NAMED(LOG, "Abstract Octree from binary msg data must be a concrete OcTree");
        return nullptr;
    }

    decltype(shapes::OcTree().octree) ot(tree);         // snap into a shared_ptr
    shapes::ShapeConstPtr sp(new shapes::OcTree(ot));   // snap into a shape

    Eigen::Isometry3d transform;
    tf::poseMsgToEigen(octomap.origin, transform);

    // construct the object
    auto o = smpl::make_unique<collision_detection::World::Object>(octomap.octomap.id); // snap into an object
    o->shapes_.push_back(sp);
    o->shape_poses_.push_back(transform);

    return std::move(o);
}

void CollisionSpaceSceneMultithread::SetCollisionSpace(CollisionSpaceMultithread* cspace)
{
    if (cspace != m_cspace) {
        // TODO: remove objects from old cspace, add objects to new cspace
        m_cspace = cspace;
    }
}

bool CollisionSpaceSceneMultithread::SetRobotState(
    int thread_idx,
    const moveit_msgs::RobotState& robot_state)
{
    if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size()) {
        ROS_ERROR("Joint state contains mismatched number of joint names and joint positions");
        return false;
    }

    for (size_t i = 0; i < robot_state.joint_state.name.size(); ++i) {
        auto& joint_name = robot_state.joint_state.name[i];
        double joint_position = robot_state.joint_state.position[i];
        if (!m_cspace->setJointPosition(thread_idx, joint_name, joint_position)) {
            ROS_ERROR("Failed to set position of joint '%s' to %f", joint_name.c_str(), joint_position);
            return false;
        }
    }

    auto& multi_dof_joint_state = robot_state.multi_dof_joint_state;
    for (size_t i = 0; i < multi_dof_joint_state.joint_names.size(); ++i) {
        auto& joint_name = multi_dof_joint_state.joint_names[i];
        auto& joint_transform = multi_dof_joint_state.transforms[i];
        // TODO: Need a way to identify what type of joint this is to extract
        // its condensed set of variables. Might be worth adding a function to
        // CollisionSpace to handle setting any joint from its local transform.
    }

    // TODO: world -> model transform should be handled by multi_dof joint
    // state.

    auto& attached_collision_objects = robot_state.attached_collision_objects;
    for (auto& aco : attached_collision_objects) {
        if (!ProcessAttachedCollisionObject(thread_idx, aco)) {
            ROS_WARN_NAMED(LOG, "Failed to process attached collision object");
            return false;
        }
    }

    return true;
}

/// \brief Process a collision object
/// \param object The collision object to be processed
/// \return true if the object was processed successfully; false otherwise
bool CollisionSpaceSceneMultithread::ProcessCollisionObjectMsg(
    int thread_idx,
    const moveit_msgs::CollisionObject& object)
{
    if (object.operation == moveit_msgs::CollisionObject::ADD) 
        return AddCollisionObjectMsg(thread_idx, object);
    else if (object.operation == moveit_msgs::CollisionObject::REMOVE)
        return RemoveCollisionObjectMsg(thread_idx, object);
    else if (object.operation == moveit_msgs::CollisionObject::APPEND)
        return AppendCollisionObjectMsg(thread_idx, object);
    else if (object.operation == moveit_msgs::CollisionObject::MOVE)
       return RemoveCollisionObjectMsg(thread_idx, object) && AddCollisionObjectMsg(thread_idx, object);
       // return MoveCollisionObjectMsg(object);
    else 
    {
        ROS_WARN_NAMED(LOG, "Collision object operation '%d' is not supported", object.operation);
        return false;
    }
}

bool CollisionSpaceSceneMultithread::AddCollisionObjectMsg(
    int thread_idx,
    const moveit_msgs::CollisionObject& object)
{
    if (m_cspace->worldCollisionModel(thread_idx)->hasObjectWithName(object.id)) {
        return false;
    }

    if (object.header.frame_id != m_cspace->getReferenceFrame()) {
        ROS_ERROR("Collision object must be specified in the Collision Space's reference frame (%s)", m_cspace->getReferenceFrame().c_str());
        return false;
    }

    if (!CheckCollisionObjectSanity(object)) {
        ROS_WARN_NAMED(LOG, "Reject addition of collision object '%s'", object.id.c_str());
        return false;
    }

    std::vector<CollisionShape*> shapes;
    AlignedVector<Eigen::Affine3d> shape_poses;

    for (size_t i = 0; i < object.primitives.size(); ++i) {
        auto& prim = object.primitives[i];

        std::unique_ptr<CollisionShape> shape;
        switch (prim.type) {
        case shape_msgs::SolidPrimitive::BOX:
            shape = smpl::make_unique<BoxShape>(
                    prim.dimensions[shape_msgs::SolidPrimitive::BOX_X],
                    prim.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
                    prim.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            shape = smpl::make_unique<SphereShape>(
                    prim.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]);
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            shape = smpl::make_unique<CylinderShape>(
                    prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
                    prim.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
            break;
        case shape_msgs::SolidPrimitive::CONE:
            shape = smpl::make_unique<ConeShape>(
                    prim.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
                    prim.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]);
            break;
        default:
            assert(0);
        }

        m_collision_shapes.push_back(std::move(shape));
        shapes.push_back(m_collision_shapes.back().get());

        auto& prim_pose = object.primitive_poses[i];
        Eigen::Affine3d transform;
        tf::poseMsgToEigen(prim_pose, transform);
        shape_poses.push_back(transform);
    }

    for (size_t i = 0; i < object.planes.size(); ++i) {
        auto& plane = object.planes[i];

        auto shape = smpl::make_unique<PlaneShape>(
                plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3]);
        m_collision_shapes.push_back(std::move(shape));
        shapes.push_back(m_collision_shapes.back().get());

        auto& plane_pose = object.plane_poses[i];
        Eigen::Affine3d transform;
        tf::poseMsgToEigen(plane_pose, transform);
        shape_poses.push_back(transform);
    }

    for (size_t i = 0; i < object.meshes.size(); ++i) {
        auto& mesh = object.meshes[i];

        auto vertices = new double[3 * mesh.vertices.size()];
        auto triangles = new std::uint32_t[3 * mesh.triangles.size()];

        int j = 0;
        for (auto vtx : mesh.vertices)
        {
            vertices[j] = 0.0254*vtx.x;
            vertices[j+1] = 0.0254*vtx.y;
            vertices[j+2] = 0.0254*vtx.z;
            j+=3;
        }
        j = 0;
        for (auto tri : mesh.triangles)
        {
            triangles[j] = tri.vertex_indices[0];
            triangles[j+1] = tri.vertex_indices[1];
            triangles[j+2] = tri.vertex_indices[2];
            j+=3;
        }

        auto ms = smpl::make_unique<MeshShape>();
        ms->vertices = vertices;
        ms->vertex_count = mesh.vertices.size();
        ms->triangles = triangles;
        ms->triangle_count = mesh.triangles.size();
        m_collision_shapes.push_back(std::move(ms));
        shapes.push_back(m_collision_shapes.back().get());
        
        auto& mesh_pose = object.mesh_poses[i];
        Eigen::Affine3d transform;
        tf::poseMsgToEigen(mesh_pose, transform);
        shape_poses.push_back(transform);
    }

    // create the collision object
    auto co = smpl::make_unique<CollisionObject>();
    co->id = object.id;
    co->shapes = std::move(shapes);
    co->shape_poses = std::move(shape_poses);

    m_collision_objects.push_back(std::move(co));

    return m_cspace->insertObject(thread_idx, m_collision_objects.back().get());

}

bool CollisionSpaceSceneMultithread::RemoveCollisionObjectMsg(
    const int thread_idx,
    const moveit_msgs::CollisionObject& object)
{
    // find the collision object with this name
    auto* _object = FindCollisionObject(object.id);
    if (!_object) {
        ROS_WARN_NAMED(LOG, "Reject removal of collision object '%s'", object.id.c_str());
        return false;
    }

    // remove from collision space
    if (!m_cspace->removeObject(thread_idx, _object)) {
        return false;
    }

    // remove all collision shapes belonging to this object
    auto belongs_to_object = [_object](const std::unique_ptr<CollisionShape>& shape) {
        auto is_shape = [&shape](const CollisionShape* s) {
            return s == shape.get();
        };
        auto it = std::find_if(
                begin(_object->shapes), end(_object->shapes), is_shape);
        return it != end(_object->shapes);
    };

    auto rit = std::remove_if(
            begin(m_collision_shapes), end(m_collision_shapes),
            belongs_to_object);
    m_collision_shapes.erase(rit, end(m_collision_shapes));

    // remove the object itself
    auto is_object = [_object](const std::unique_ptr<CollisionObject>& object) {
        return object.get() == _object;
    };
    auto rrit = std::remove_if(
            begin(m_collision_objects), end(m_collision_objects), is_object);
    m_collision_objects.erase(rrit, end(m_collision_objects));
    return true;
}

bool CollisionSpaceSceneMultithread::AppendCollisionObjectMsg(
    const int thread_idx,
    const moveit_msgs::CollisionObject& object)
{
    auto* _object = FindCollisionObject(object.id);
    if (!_object) {
        ROS_WARN_NAMED(LOG, "Reject append to missing collision object '%s'", object.id.c_str());
        return false;
    }

    return m_cspace->insertShapes(thread_idx, _object);
}

bool CollisionSpaceSceneMultithread::MoveCollisionObjectMsg(
    int thread_idx,
    const moveit_msgs::CollisionObject& object)
{
    auto* _object = FindCollisionObject(object.id);
    if (!_object) {
        ROS_WARN_NAMED(LOG, "Rejecting move of missing collision object '%s'", object.id.c_str());
        return false;
    }

    return m_cspace->moveShapes(thread_idx, _object);
}

/// \brief Process an octomap
/// \param octomap The octomap
/// \return true if the octomap was processed successfully; false otherwise
// bool CollisionSpaceSceneMultithread::ProcessOctomapMsg(
//     const octomap_msgs::OctomapWithPose& octomap)
// {
//     if (!CheckInsertOctomap(octomap)) {
//         ROS_WARN_NAMED(LOG, "Rejecting addition of octomap '%s'", octomap.octomap.id.c_str());
//         return false;
//     }

//     // convert binary octomap message to octree
//     octomap::AbstractOcTree* abstract_tree =
//             octomap_msgs::binaryMsgToMap(octomap.octomap);
//     if (!abstract_tree) {
//         ROS_WARN_NAMED(LOG, "Failed to convert binary msg data to octomap");
//         return false;
//     }

//     octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
//     if (!tree) {
//         ROS_WARN_NAMED(LOG, "Abstract Octree from binary msg data must be a concrete OcTree");
//         return false;
//     }

//     std::unique_ptr<octomap::OcTree> ot(tree);

//     auto shape = smpl::make_unique<OcTreeShape>(ot.get());

//     Eigen::Affine3d transform;
//     tf::poseMsgToEigen(octomap.origin, transform);

//     auto object = smpl::make_unique<CollisionObject>();
//     object->id = octomap.octomap.id;
//     object->shapes.push_back(shape.get());
//     object->shape_poses.push_back(transform);

//     if (m_cspace->insertObject(object.get())) {
//         m_octrees.push_back(std::move(ot));
//         m_collision_shapes.push_back(std::move(shape));
//         m_collision_objects.push_back(std::move(object));
//         return true;
//     } else {
//         ROS_WARN_NAMED(LOG, "Failed to insert OcTree into Collision Space");
//     }
//     return false;
// }

/// \brief Process an attached collision object
/// \param ao The attached collision object
/// \return true if the attached collision object was processed successfully;
///     false otherwise
bool CollisionSpaceSceneMultithread::ProcessAttachedCollisionObject(
    int thread_idx,
    const moveit_msgs::AttachedCollisionObject& ao)
{
    switch (ao.object.operation) {
    case moveit_msgs::CollisionObject::ADD:
    {
        auto o = ConvertCollisionObjectToObjectMultithread(ao.object);
        if (!m_cspace->attachObject(thread_idx,
                ao.object.id, o->shapes_, o->shape_poses_, ao.link_name))
        {
            return false;
        }

        // TODO: handle touch links
        return true;
    }   break;
    case moveit_msgs::CollisionObject::REMOVE:
    {
        if (!m_cspace->detachObject(thread_idx, ao.object.id)) {
            return false;
        }
        return true;
    }   break;
    case moveit_msgs::CollisionObject::APPEND:
    case moveit_msgs::CollisionObject::MOVE:
    default:
        assert(0);
    }

    return false;
}

// bool CollisionSpaceSceneMultithread::UpdatePlanningSceneWorld(
//     const moveit_msgs::PlanningSceneWorld& world,
//     bool is_diff)
// {
//     if (!is_diff) {
//         // TODO: for full planning scenes, remove all collision objects from the
//         // world collision model
//         return false;
//     }

//     ROS_INFO_NAMED(LOG, "Processing %zd collision objects", world.collision_objects.size());
//     for (auto& collision_object : world.collision_objects) {
//         if (!ProcessCollisionObjectMsg(collision_object)) {
//             ROS_WARN_NAMED(LOG, "Failed to process collision object '%s'", collision_object.id.c_str());
//             return false;
//         }
//     }

//     auto& octomap = world.octomap;
//     if (!ProcessOctomapMsg(octomap)) {
//         ROS_WARN_NAMED(LOG, "Failed to process octomap '%s'", octomap.octomap.id.c_str());
//         return false;
//     }

//     return true;
// }

// bool CollisionSpaceSceneMultithread::UpdatePlanningScene(
//     int thread_idx,
//     const moveit_msgs::PlanningScene& scene)
// {
//     ROS_INFO_NAMED(LOG, "Updating the Collision Space from Planning Scene '%s'", scene.name.c_str());

//     // TODO: currently ignored fields from moveit_msgs::PlanningScene
//     // * fixed_frame_transforms
//     // * link_padding
//     // * link_scale
//     // * object_colors

//     if (scene.robot_model_name != m_cspace->robotCollisionModel()->name()) {
//         ROS_WARN_NAMED(LOG, "Planning Scene is for a different robot");
//         return false;
//     }

//     if (!SetRobotState(thread_idx, scene.robot_state)) {
//         ROS_WARN_NAMED(LOG, "Failed to update robot state");
//         return false;
//     }

//     AllowedCollisionMatrix acm(scene.allowed_collision_matrix);
//     if (scene.is_diff) {
//         m_cspace->updateAllowedCollisionMatrix(acm);
//     } else {
//         m_cspace->setAllowedCollisionMatrix(acm);
//     }

//     if (!UpdatePlanningSceneWorld(scene.world, scene.is_diff)) {
//         return false;
//     }

//     return true;
// }

// Return true if this collision object is suitable to be added to the world
// collision model. Note this does not yet check for duplicates, since they are
// rejected immediately before being inserted.
bool CollisionSpaceSceneMultithread::CheckCollisionObjectSanity(
    const moveit_msgs::CollisionObject& object) const
{
    if (object.primitives.size() != object.primitive_poses.size()) {
        ROS_ERROR("Mismatched sizes of primitives and primitive poses");
        return false;
    }

    if (object.meshes.size() != object.mesh_poses.size()) {
        ROS_ERROR("Mismatches sizes of meshes and mesh poses");
        return false;
    }

    // check solid primitive for correct format
    for (auto& prim : object.primitives) {
        switch (prim.type) {
        case shape_msgs::SolidPrimitive::BOX:
        {
            if (prim.dimensions.size() != 3) {
                ROS_ERROR("Invalid number of dimensions for box of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 3, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::SPHERE:
        {
            if (prim.dimensions.size() != 1) {
                ROS_ERROR("Invalid number of dimensions for sphere of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 1, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CYLINDER:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR("Invalid number of dimensions for cylinder of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CONE:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR("Invalid number of dimensions for cone of collision object '%s' (Expected: %d, Actual: %zu)", object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        default:
            ROS_ERROR("Unrecognized SolidPrimitive type");
            return false;
        }
    }

    return true;
}

// bool CollisionSpaceSceneMultithread::CheckInsertOctomap(
//     const octomap_msgs::OctomapWithPose& octomap) const
// {
//     if (octomap.header.frame_id != m_cspace->getReferenceFrame()) {
//         ROS_WARN_NAMED(LOG, "Octomap must be specified in the grid reference frame (%s)", m_cspace->getReferenceFrame().c_str());
//         return false;
//     }

//     if (!octomap.octomap.binary) {
//         ROS_WARN_NAMED(LOG, "Octomap must be a binary octomap");
//         return false;
//     }

//     return true;
// }

// auto CollisionSpaceSceneMultithread::FindCollisionObject(const std::string& id) const
//     -> CollisionObject*
// {
//     for (auto& object : m_collision_objects) {
//         if (object->id == id) {
//             return object.get();
//         }
//     }
//     return nullptr;
// }
