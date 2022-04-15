#ifndef DARTROBOTS_WORLD_HPP
#define DARTROBOTS_WORLD_HPP
#include <Eigen/Core>
#include <memory>
#include <string>
namespace DartRobots
{
class MiniCheetah;
class World
{
  public:
    World();
    ~World();
    void Step(uint64_t iter);
    void Reset();
    void Render();
    void SetRobot(std::shared_ptr<MiniCheetah> robot);
    void SetTerrainUrdf(const std::string &urdfPath = "");
    std::string AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                        const std::string &name = "marker");
    bool SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation);
    void DeleteBall(const std::string &name);

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace DartRobots
#endif // DARTROBOTS_WORLD_HPP
