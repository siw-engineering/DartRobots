#ifndef DARTROBOTS_MINICHEETAH_HPP
#define DARTROBOTS_MINICHEETAH_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace DartRobots
{

struct MiniCheetahConfig
{
    Eigen::Vector3d spawnPos{0.0, 0.0, 0.3};
    Eigen::Vector3d spawnOrientation{0.0, 0.0, 0.0};
    Eigen::Matrix<double, 12, 1> spawnJointPos{
        (Eigen::Matrix<double, 12, 1>() << 0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6, 0, -0.8, 1.6).finished()};
};

class MiniCheetah
{
  public:
    explicit MiniCheetah(const MiniCheetahConfig &config = MiniCheetahConfig{});
    ~MiniCheetah();
    void Step(uint64_t iter);
    void Render();
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
    [[nodiscard]] Eigen::Vector3d GetWorldLinVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldAngVel() const;
    [[nodiscard]] Eigen::Vector3d GetWorldLinAcc() const;
    std::string AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                        const std::string &name = "marker");
    bool SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation, const std::string &frame="");
    void DeleteBall(const std::string &name);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace DartRobots

#endif // DARTROBOTS_MINICHEETAH_HPP
