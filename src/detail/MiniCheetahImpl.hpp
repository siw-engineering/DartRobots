#ifndef DARTROBOTS_DETAIL_MINICHEETAHIMPL_HPP
#define DARTROBOTS_DETAIL_MINICHEETAHIMPL_HPP
#include "DartRobots/MiniCheetah.hpp"
#include <chrono>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>
#include <dart/simulation/SmartPointer.hpp>

namespace DartRobots
{
class MiniCheetah::Impl
{
  public:
    explicit Impl(MiniCheetahConfig config);
    void Reset();
    void SetContactDirty();
    void SetWorld(dart::simulation::ConstWorldPtr world);

    void SetJointCommands(Eigen::Matrix<double, 12, 1> commands);
    void SetCommandType(CommandType cmdType);

    dart::dynamics::SkeletonPtr GetSkeleton() const;
    void SaveState(unsigned checkpointId);
    void LoadState(unsigned checkpointId);
    void SetJointCoulombFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val);
    void SetJointViscousFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val);
    void SetFootFriction(Eigen::Ref<const Eigen::Matrix<double, 4, 1>> val);
    Eigen::Matrix<double, 12, 1> GetJointCoulombFriction() const;
    Eigen::Matrix<double, 12, 1> GetJointViscousFriction() const;
    Eigen::Matrix<double, 4, 1> GetFootFriction() const;
    Eigen::Matrix<double, 3, 4> GetFootPositions() const;
    Eigen::Matrix<bool, 4, 1> GetFootContactStates() const;
    Eigen::Matrix<double, 3, 4> GetFootContactForces() const;
    Eigen::Matrix<double, 3, 4> GetFootContactNormals() const;
    Eigen::Matrix<double, 12, 1> GetJointPositions() const;
    Eigen::Matrix<double, 12, 1> GetJointVelocities() const;
    [[nodiscard]] Eigen::Quaterniond GetOrientation() const;
    [[nodiscard]] Eigen::Vector3d GetBodyPosition() const;
    [[nodiscard]] Eigen::Vector3d GetWorldLinVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldAngVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldLinAcc() const;

  private:
    void UpdateContactData() const;
    MiniCheetahConfig config_;

    dart::simulation::ConstWorldPtr world_;
    std::vector<dart::dynamics::Joint *> revoluteJoints_{};
    std::shared_ptr<dart::dynamics::Skeleton> robot_{nullptr};


    std::unordered_map<const dart::dynamics::Shape *, uint32_t> collShapeLegIndexMap_{};
    std::array<dart::dynamics::BodyNode *, 4> legNodes_{};

    std::unordered_map<unsigned, dart::dynamics::Skeleton::Configuration> configMap_{};

    // We store buffers here to optimise data retrieval times
    Eigen::Matrix<double, 12, 1> coulombFriction_{};
    Eigen::Matrix<double, 12, 1> viscousFriction_{};
    Eigen::Matrix<double, 4, 1> footFriction_{};

    mutable bool contactDataDirty_{true};
    mutable Eigen::Matrix<bool, 4, 1> footContactStates_{Eigen::Matrix<bool, 4, 1>::Constant(false)};
    mutable Eigen::Matrix<double, 3, 4> footContactForces_{Eigen::Matrix<double, 3, 4>::Zero()};
    mutable Eigen::Matrix<double, 3, 4> footContactNormals_{Eigen::Matrix<double, 3, 4>::Zero()};
};

} // namespace DartRobots

#endif // DARTROBOTS_DETAIL_MINICHEETAHIMPL_HPP
