#ifndef DARTROBOTS_DETAIL_WORLDIMPL_HPP
#define DARTROBOTS_DETAIL_WORLDIMPL_HPP

#include "DartRobots/MiniCheetah.hpp"
#include "DartRobots/World.hpp"
#include <dart/dynamics/Skeleton.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/WorldNode.hpp>
#include <dart/simulation/World.hpp>

namespace DartRobots
{
class World::Impl
{
  public:
    Impl();
    void Step(uint64_t iter);
    void Reset();
    void Render();
    void SetRobot(std::shared_ptr<MiniCheetah> robot);
    std::string ChangeTerrain();
    std::string AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                        const std::string &name = "marker");
    bool SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation);
    void DeleteBall(const std::string &name);

  private:
    std::chrono::time_point<std::chrono::steady_clock> startTime_{};
    double simTimeElapsed_{0.0};
    double realTimeElapsed_{0.0};

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr terrain_;
    std::shared_ptr<MiniCheetah> robot_{nullptr};

    ::osg::ref_ptr<dart::gui::osg::WorldNode> node_{nullptr};
    std::unique_ptr<dart::gui::osg::ImGuiViewer> viewer_{nullptr};
};
} // namespace DartRobots

#endif // DARTROBOTS_DETAIL_WORLDIMPL_HPP
