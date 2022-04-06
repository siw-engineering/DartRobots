#include "DartRobots/MiniCheetah.hpp"
#include "detail/MiniCheetahImpl.hpp"

namespace DartRobots
{

MiniCheetah::MiniCheetah(const MiniCheetahConfig &config) : impl_(std::make_unique<Impl>(config))
{
}
MiniCheetah::~MiniCheetah() = default;

void MiniCheetah::SetJointCommands(const Eigen::Matrix<double, 12, 1> &commands)
{
    impl_->SetJointCommands(commands);
}
void MiniCheetah::Step(uint64_t iter)
{
    impl_->Step(iter);
}
void MiniCheetah::Render()
{
    impl_->Render();
}
void MiniCheetah::SaveState(unsigned int checkpointId)
{
    impl_->SaveState(checkpointId);
}
void MiniCheetah::LoadState(unsigned int checkpointId)
{
    impl_->LoadState(checkpointId);
}
void MiniCheetah::SetJointCoulombFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val)
{
    impl_->SetJointCoulombFriction(val);
}
void MiniCheetah::SetJointViscousFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val)
{
    impl_->SetJointViscousFriction(val);
}
void MiniCheetah::SetFootFriction(Eigen::Ref<const Eigen::Matrix<double, 4, 1>> val)
{
    impl_->SetFootFriction(val);
}
Eigen::Matrix<double, 12, 1> MiniCheetah::GetJointCoulombFriction() const
{
    return impl_->GetJointCoulombFriction();
}
Eigen::Matrix<double, 12, 1> MiniCheetah::GetJointViscousFriction() const
{
    return impl_->GetJointViscousFriction();
}
Eigen::Matrix<double, 4, 1> MiniCheetah::GetFootFriction() const
{
    return impl_->GetFootFriction();
}
Eigen::Matrix<double, 3, 4> MiniCheetah::GetFootPositions() const
{
    return impl_->GetFootPositions();
}
Eigen::Matrix<bool, 4, 1> MiniCheetah::GetFootContactStates() const
{
    return impl_->GetFootContactStates();
}
Eigen::Matrix<double, 3, 4> MiniCheetah::GetFootContactForces() const
{
    return impl_->GetFootContactForces();
}
Eigen::Matrix<double, 3, 4> MiniCheetah::GetFootContactNormals() const
{
    return impl_->GetFootContactNormals();
}
Eigen::Matrix<double, 12, 1> MiniCheetah::GetJointPositions() const
{
    return impl_->GetJointPositions();
}
Eigen::Matrix<double, 12, 1> MiniCheetah::GetJointVelocities() const
{
    return impl_->GetJointVelocities();
}
Eigen::Quaterniond MiniCheetah::GetOrientation() const
{
    return impl_->GetOrientation();
}
Eigen::Vector3d MiniCheetah::GetWorldLinVel() const
{
    return impl_->GetWorldLinVel();
}
Eigen::Vector3d MiniCheetah::GetWorldAngVel() const
{
    return impl_->GetWorldAngVel();
}
Eigen::Vector3d MiniCheetah::GetWorldLinAcc() const
{
    return impl_->GetWorldLinAcc();
}
} // namespace DartRobots