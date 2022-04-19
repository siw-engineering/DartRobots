#include "DartRobots/World.hpp"
#include "detail/WorldImpl.hpp"

namespace DartRobots
{
World::World() : impl_(std::make_unique<Impl>())
{
}
World::~World() = default;

void World::Step(uint64_t iter)
{
    impl_->Step(iter);
}
void World::Reset()
{
    impl_->Reset();
}
void World::Render()
{
    impl_->Render();
}
void World::SetRobot(std::shared_ptr<MiniCheetah> robot)
{
    impl_->SetRobot(std::move(robot));
}

void World::SetTerrain(Terrain terrain)
{
    impl_->SetTerrain(terrain);
}

void World::SetTerrainUrdf(const std::string &urdfPath)
{
    impl_->SetTerrainUrdf(urdfPath);
}
std::string World::AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                           const std::string &name)
{
    return impl_->AddBall(translation, color, radius, name);
}
bool World::SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation)
{
    return impl_->SetBallTranslation(name, translation);
}
void World::DeleteBall(const std::string &name)
{
    impl_->DeleteBall(name);
}
} // namespace DartRobots
