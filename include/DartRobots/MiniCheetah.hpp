#ifndef DARTROBOTS_MINICHEETAH_HPP
#define DARTROBOTS_MINICHEETAH_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace dart::simulation
{
class World;
} // namespace dart::simulation
namespace dart::dynamics
{
class Skeleton;
} // namespace dart::dynamics

namespace DartRobots
{

struct MiniCheetahConfig
{
    Eigen::Vector3d spawnPos{0.0, 0.0, 0.3};
    Eigen::Vector3d spawnOrientation{0.0, 0.0, 0.0};
    Eigen::Matrix<double, 12, 1> spawnJointPos{
        (Eigen::Matrix<double, 12, 1>() << 0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6).finished()};
    std::string urdfPath{};
};

class MiniCheetah
{
  public:
    explicit MiniCheetah(const MiniCheetahConfig &config = MiniCheetahConfig{});
    ~MiniCheetah();
    void Reset();
    void SetContactDirty();
    void SetWorld(std::shared_ptr<dart::simulation::World> world);
    [[nodiscard]] std::shared_ptr<dart::dynamics::Skeleton> GetSkeleton() const;

    void SaveState(unsigned checkpointId);
    void LoadState(unsigned checkpointId);
    void SetJointCommands(const Eigen::Matrix<double, 12, 1> &commands);
    // TODO: Add set joint mode to allow both torque and velocity control
    void SetJointCoulombFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val);
    void SetJointViscousFriction(Eigen::Ref<const Eigen::Matrix<double, 12, 1>> val);
    void SetFootFriction(Eigen::Ref<const Eigen::Matrix<double, 4, 1>> val);

    [[nodiscard]] Eigen::Matrix<double, 12, 1> GetJointCoulombFriction() const;
    [[nodiscard]] Eigen::Matrix<double, 12, 1> GetJointViscousFriction() const;
    [[nodiscard]] Eigen::Matrix<double, 4, 1> GetFootFriction() const;

    /**
     * Gets absolute foot position in world frame, note not to use this wrongly
     * @return
     */
    [[nodiscard]] Eigen::Matrix<double, 3, 4> GetFootPositions() const;

    [[nodiscard]] Eigen::Matrix<bool, 4, 1> GetFootContactStates() const;
    [[nodiscard]] Eigen::Matrix<double, 3, 4> GetFootContactForces() const;
    [[nodiscard]] Eigen::Matrix<double, 3, 4> GetFootContactNormals() const;

    [[nodiscard]] Eigen::Matrix<double, 12, 1> GetJointPositions() const;
    [[nodiscard]] Eigen::Matrix<double, 12, 1> GetJointVelocities() const;
    [[nodiscard]] Eigen::Quaterniond GetOrientation() const;
    [[nodiscard]] Eigen::Vector3d GetBodyPosition() const;
    [[nodiscard]] Eigen::Vector3d GetWorldLinVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldAngVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldLinAcc() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace DartRobots

#endif // DARTROBOTS_MINICHEETAH_HPP
