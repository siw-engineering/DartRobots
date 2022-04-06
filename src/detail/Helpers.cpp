#include "Helpers.hpp"
#include <dart/dynamics/dynamics.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <spdlog/spdlog.h>

using namespace dart::dynamics;
namespace DartRobots
{

void SetPose(dart::dynamics::Skeleton *robot, const Eigen::Vector3d &pos, const Eigen::Vector3d &orientation)
{
    // Set Orientation
    robot->setPosition(0, orientation[0]);
    robot->setPosition(1, orientation[1]);
    robot->setPosition(2, orientation[2]);
    // Set position
    robot->setPosition(3, pos[0]);
    robot->setPosition(4, pos[1]);
    robot->setPosition(5, pos[2]);
}
void SetRevoluteJointPositions(dart::dynamics::Skeleton *robot, tcb::span<double> jointPositions)
{
    auto joints = robot->getJoints();
    auto i = 0; // NOLINT(readability-identifier-length)
    for (auto &joint : joints)
    {
        const auto &jointType = joint->getType();
        if (jointType == "RevoluteJoint")
        {
            joint->setPosition(0, jointPositions[i]);
            i++;
        }
    }
}
std::vector<Joint *> ReadRevoluteJoints(Skeleton *robot, unsigned expectedSize)
{
    std::vector<Joint *> buffer{};
    auto joints = robot->getJoints();
    for (auto &joint : joints)
    {
        const auto &jointType = joint->getType();
        if (jointType == "RevoluteJoint")
        {
            buffer.emplace_back(joint);
        }
    }
    if (buffer.size() != expectedSize)
    {
        std::string msg = fmt::format("Revolute joint count not {}}, count: {}", expectedSize, buffer.size());
        spdlog::error(msg);
        throw std::runtime_error(msg);
    }
    return buffer;
}
void PrintRobotSkeletonInfo(Skeleton *robot)
{
    auto dofs = robot->getDofs();
    for (auto &dof : dofs)
    {
        std::string temp = fmt::format("Dof {}: {}", dof->getIndexInTree(), dof->getName());
        spdlog::debug(temp);
    }
    auto bodyNodes = robot->getBodyNodes();
    for (auto *bodyNode : bodyNodes)
    {
        auto nodeName = bodyNode->getName();
        std::string temp = fmt::format("Body node {}: {}", bodyNode->getIndexInTree(), nodeName);
        spdlog::debug(temp);
    }
}
std::array<BodyNode *, 4> GetLegNodes(Skeleton *robot, const std::array<std::string, 4> &footLinkNames)
{
    std::array<BodyNode *, 4> temp{};
    auto bodyNodes = robot->getBodyNodes();
    for (auto *bodyNode : bodyNodes)
    {
        auto nodeName = bodyNode->getName();
        for (unsigned i = 0; i < 4; i++)
        {
            if (nodeName == footLinkNames.at(i))
            {
                temp.at(i) = bodyNode;
                spdlog::trace("Node with name {} added to legNodes with index {}", nodeName, i);
            }
        }
    }
    return temp;
}

std::unordered_map<const Shape *, uint32_t> GetShapeToLegIndexMap(const Skeleton *robot,
                                                                  const std::array<std::string, 4> &footLinkNames)
{
    std::unordered_map<const Shape *, uint32_t> temp;
    auto bodyNodes = robot->getBodyNodes();
    for (const auto *bodyNode : bodyNodes)
    {
        auto nodeName = bodyNode->getName();
        for (unsigned i = 0; i < 4; i++)
        {
            if (nodeName == footLinkNames.at(i))
            {
                auto collShapeNodes = bodyNode->getShapeNodesWith<CollisionAspect>();
                for (const auto *collShapeNode : collShapeNodes)
                {
                    const auto *ptr = collShapeNode->getShape().get();
                    temp[ptr] = i;
                    spdlog::trace("Shape with addr {} assigned to leg index {}", reinterpret_cast<uintptr_t>(ptr), i);
                }
            }
        }
    }
    return temp;
}
} // namespace DartRobots