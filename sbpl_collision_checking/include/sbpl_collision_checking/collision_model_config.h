////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef sbpl_collision_CollisionModelConfig_h
#define sbpl_collision_CollisionModelConfig_h

// standard includes
#include <ostream>
#include <string>
#include <vector>

// system includes
#include <moveit/collision_detection/collision_matrix.h>
#include <ros/ros.h>

namespace sbpl {
namespace collision {

struct CollisionSphereConfig
{
    std::string name;
    double x;
    double y;
    double z;
    double radius;
    int priority;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionSphereConfig& cfg);
};

std::ostream& operator<<(std::ostream&, const CollisionSphereConfig&);

struct CollisionSpheresModelConfig
{
    std::string link_name;
    std::vector<std::string> spheres;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionSpheresModelConfig& cfg);
};

std::ostream& operator<<(std::ostream&, const CollisionSpheresModelConfig&);

struct CollisionVoxelModelConfig
{
    std::string link_name;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionVoxelModelConfig& cfg);
};

std::ostream& operator<<(std::ostream&, const CollisionVoxelModelConfig&);

struct CollisionGroupConfig
{
    std::string name;
    std::vector<std::string> links;

    static bool Load(XmlRpc::XmlRpcValue& config, CollisionGroupConfig& cfg);
};

std::ostream& operator<<(std::ostream&, const CollisionGroupConfig&);

struct CollisionModelConfig
{
    std::vector<CollisionSphereConfig>          spheres;
    std::vector<CollisionSpheresModelConfig>    spheres_models;
    std::vector<CollisionVoxelModelConfig>      voxel_models;
    std::vector<CollisionGroupConfig>           groups;
    collision_detection::AllowedCollisionMatrix acm;

    static bool Load(const ros::NodeHandle& nh, CollisionModelConfig& cfg);
};

} // namespace collision
} // namespace sbpl

#endif
