#include "MiniCheetahImpl.hpp"
#include "DartRobots/Config.hpp"
#include "Helpers.hpp"
#include <chrono>
#include <dart/collision/ode/ode.hpp>
#include <dart/constraint/constraint.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <spdlog/spdlog.h>
#include <utility>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace std::chrono;
using namespace std::chrono_literals;

namespace DartRobots
{

MiniCheetah::Impl::Impl(MiniCheetahConfig config) : config_(std::move(config))
{
    auto robotUri = dart::common::Uri();
    if (config_.urdfPath.empty())
    {
        const std::string robotName = "mini_cheetah";
        const std::string robotUrdfPath =
            fmt::format("{}{}/{}.urdf", Config::install_robots_path, robotName, robotName);
        robotUri.fromPath(robotUrdfPath);
    }
    else
    {
        robotUri.fromPath(config_.urdfPath);
    }
    spdlog::debug("robot uri path: {}", robotUri.getPath());

    DartLoader urdfLoader;
    robot_ = urdfLoader.parseSkeleton(robotUri);
    robot_->disableSelfCollisionCheck();
    PrintRobotSkeletonInfo(robot_.get());
    legNodes_ = GetLegNodes(robot_.get(), {"toe_fl", "toe_fr", "toe_hl", "toe_hr"});
    collShapeLegIndexMap_ = GetShapeToLegIndexMap(robot_.get(), {"toe_fl", "toe_fr", "toe_hl", "toe_hr"});

    SetFootFriction(Eigen::Matrix<double, 4, 1>::Constant(0.4));
    revoluteJoints_ = ReadRevoluteJoints(robot_.get());
    SetPose(robot_.get(), config_.spawnPos, config_.spawnOrientation);
    SetRevoluteJointPositions(robot_.get(), tcb::span<double>{config_.spawnJointPos.data(), 12});
    for (int i = 0; i < 12; i++)
    {
        revoluteJoints_.at(i)->setActuatorType(dart::dynamics::Joint::FORCE);
    }
}
void MiniCheetah::Impl::SetJointCommands(Eigen::Matrix<double, 12, 1> commands)
{
    for (unsigned i = 0; i < 12; i++)
    {
        revoluteJoints_.at(i)->setCommand(0, commands(i));
    }
}

void MiniCheetah::Impl::Reset()
{
    coulombFriction_ = Eigen::Matrix<double, 12, 1>::Zero();
    viscousFriction_ = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 4; i++)
    {
        footFriction_(i) = legNodes_.at(i)->getFrictionCoeff();
    }
    contactDataDirty_ = true;
}

void MiniCheetah::Impl::SetContactDirty()
{
    contactDataDirty_ = true;
}

void MiniCheetah::Impl::SetWorld(ConstWorldPtr world)
{
    world_ = std::move(world);
}

void MiniCheetah::Impl::SaveState(unsigned checkpointId)
{
    configMap_[checkpointId] = robot_->getConfiguration();
}

void MiniCheetah::Impl::LoadState(unsigned checkpointId)
{
    if (configMap_.find(checkpointId) != configMap_.end())
    {
        robot_->setConfiguration(configMap_.at(checkpointId));
    }
}

void MiniCheetah::Impl::SetJointCoulombFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val)
{
    coulombFriction_ = val;
    for (unsigned i = 0; i < 12; i++)
    {
        revoluteJoints_.at(i)->setCoulombFriction(0, val(i));
    }
}

void MiniCheetah::Impl::SetJointViscousFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val)
{
    viscousFriction_ = val;
    for (unsigned i = 0; i < 12; i++)
    {
        revoluteJoints_.at(i)->setDampingCoefficient(0, val(i));
    }
}

void MiniCheetah::Impl::SetFootFriction(Eigen::Ref<const Eigen::Matrix<double, 4, 1>> val)
{
    footFriction_ = val;
    for (unsigned i = 0; i < 4; i++)
    {
        legNodes_.at(i)->setFrictionCoeff(val(i));
    }
}

Eigen::Matrix<double, 12, 1> MiniCheetah::Impl::GetJointCoulombFriction() const
{
    return coulombFriction_;
}

Eigen::Matrix<double, 12, 1> MiniCheetah::Impl::GetJointViscousFriction() const
{
    return viscousFriction_;
}

Eigen::Matrix<double, 4, 1> MiniCheetah::Impl::GetFootFriction() const
{
    return footFriction_;
}

Eigen::Matrix<double, 3, 4> MiniCheetah::Impl::GetFootPositions() const
{
    Eigen::Matrix<double, 3, 4> temp{};
    for (int i = 0; i < 4; i++)
    {
        temp.block<3, 1>(0, i) = legNodes_.at(i)->getCOM();
    }
    return temp;
}

Eigen::Matrix<bool, 4, 1> MiniCheetah::Impl::GetFootContactStates() const
{
    UpdateContactData();
    return footContactStates_;
}

Eigen::Matrix<double, 3, 4> MiniCheetah::Impl::GetFootContactForces() const
{
    UpdateContactData();
    return footContactForces_;
}

Eigen::Matrix<double, 3, 4> MiniCheetah::Impl::GetFootContactNormals() const
{
    UpdateContactData();
    return footContactNormals_;
}

Eigen::Matrix<double, 12, 1> MiniCheetah::Impl::GetJointPositions() const
{
    Eigen::Matrix<double, 12, 1> temp{};
    for (unsigned i = 0; i < 12; i++)
    {
        const auto *joint = revoluteJoints_.at(i);
        temp(i) = joint->getPosition(0);
    }
    return temp;
}

Eigen::Matrix<double, 12, 1> MiniCheetah::Impl::GetJointVelocities() const
{
    Eigen::Matrix<double, 12, 1> temp{};
    for (unsigned i = 0; i < 12; i++)
    {
        const auto *joint = revoluteJoints_.at(i);
        temp(i) = joint->getVelocity(0);
    }
    return temp;
}

Eigen::Quaterniond MiniCheetah::Impl::GetOrientation() const
{
    return Eigen::Quaterniond(robot_->getBodyNode(0)->getWorldTransform().rotation());
}

Eigen::Vector3d MiniCheetah::Impl::GetBodyPosition() const
{
    return robot_->getBodyNode(0)->getCOM();
}
Eigen::Vector3d MiniCheetah::Impl::GetWorldLinVel() const
{
    return robot_->getBodyNode(0)->getLinearVelocity();
}
Eigen::Vector3d MiniCheetah::Impl::GetWorldAngVel() const
{
    return robot_->getBodyNode(0)->getAngularVelocity();
}
Eigen::Vector3d MiniCheetah::Impl::GetWorldLinAcc() const
{
    return robot_->getBodyNode(0)->getLinearAcceleration();
}

void MiniCheetah::Impl::UpdateContactData() const
{
    if (!contactDataDirty_)
        return;
    // Get contact states
    Eigen::Matrix<bool, 4, 1> footContactStates = Eigen::Matrix<bool, 4, 1>::Constant(false);
    Eigen::Matrix<double, 3, 4> footContactForces = Eigen::Matrix<double, 3, 4>::Zero();
    Eigen::Matrix<double, 3, 4> footContactNormals = Eigen::Matrix<double, 3, 4>::Zero();
    auto collisionResult = world_->getLastCollisionResult();
    auto contacts = collisionResult.getContacts();
    for (auto &contact : contacts)
    {
        const auto *shape1 = contact.collisionObject1->getShape().get();
        const auto *shape2 = contact.collisionObject2->getShape().get();
        uint32_t index; // NOLINT(cppcoreguidelines-init-variables)
        if (collShapeLegIndexMap_.find(shape1) != collShapeLegIndexMap_.end())
        {
            index = collShapeLegIndexMap_.at(shape1);
        }
        else if (collShapeLegIndexMap_.find(shape2) != collShapeLegIndexMap_.end())
        {
            index = collShapeLegIndexMap_.at(shape2);
        }
        else
        {
            continue;
        }
        assert(index <= 3); // NOLINT(hicpp-no-array-decay)
        footContactStates(index) = true;
        footContactForces.block<3, 1>(0, index) = contact.force;
        footContactNormals.block<3, 1>(0, index) = contact.normal;
    }
    footContactStates_ = footContactStates;
    footContactNormals_ = footContactNormals;
    footContactForces_ = footContactForces;
    contactDataDirty_ = false;
}
dart::dynamics::SkeletonPtr MiniCheetah::Impl::GetSkeleton() const
{
    return robot_;
}

} // namespace DartRobots