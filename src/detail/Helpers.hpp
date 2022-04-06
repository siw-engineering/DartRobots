#ifndef DARTROBOTS_DETAIL_HELPERS_HPP
#define DARTROBOTS_DETAIL_HELPERS_HPP

#include "DartRobots/span.hpp"
#include <Eigen/Core>
#include <array>
#include <unordered_map>
#include <vector>

namespace dart::dynamics // NOLINT(readability-identifier-naming)
{
class Skeleton;
class BodyNode;
class Shape;
class Joint;
} // namespace dart::dynamics

namespace DartRobots
{
void SetPose(dart::dynamics::Skeleton *robot, const Eigen::Vector3d &pos, const Eigen::Vector3d &orientation);
void SetRevoluteJointPositions(dart::dynamics::Skeleton *robot, tcb::span<double> jointPositions);
std::vector<dart::dynamics::Joint *> ReadRevoluteJoints(dart::dynamics::Skeleton *robot, unsigned expectedSize = 12);
void PrintRobotSkeletonInfo(dart::dynamics::Skeleton *robot);
std::array<dart::dynamics::BodyNode *, 4> GetLegNodes(dart::dynamics::Skeleton *robot,
                                                      const std::array<std::string, 4> &footLinkNames);
std::unordered_map<const dart::dynamics::Shape *, uint32_t> GetShapeToLegIndexMap(
    const dart::dynamics::Skeleton *robot, const std::array<std::string, 4> &footLinkNames);
} // namespace DartRobots

#endif // DARTROBOTS_DETAIL_HELPERS_HPP
