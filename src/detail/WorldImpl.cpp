#include "WorldImpl.hpp"
#include "DartRobots/Config.hpp"
#include <chrono>
#include <dart/collision/ode/ode.hpp>
#include <dart/common/Uri.hpp>
#include <dart/constraint/constraint.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <filesystem>
#include <spdlog/spdlog.h>


namespace {
dart::dynamics::SkeletonPtr DartTerrainFromData(Terrain terrain)
{

    auto config = terrain.config;
    // create shape of terrrain
    auto terrainShape = std::make_shared<dart::dynamics::HeightmapShape<float>>();
    auto scale = Eigen::Vector3f{config.resolution,
                                 config.resolution,
                                 1.0};

    auto xs = int(config.xSize / config.resolution) + 1;
    auto ys = int(config.ySize / config.resolution) + 1;

    terrainShape->setHeightField(xs,
                                 ys,
                                 terrain.heights);
    terrainShape->setScale(scale);
    // Make skeleton
    dart::dynamics::SkeletonPtr terrainSkel =
        dart::dynamics::Skeleton::create();

    dart::dynamics::BodyNodePtr terrainBodyCollision
        = terrainSkel->createJointAndBodyNodePair<
                         dart::dynamics::WeldJoint>(nullptr).second;

    Eigen::Isometry3d tf_trans = Eigen::Isometry3d::Identity();
    tf_trans.translation() = Eigen::Vector3d{0.0, 0.0, 0.0};
    terrainBodyCollision->getParentJoint()->setTransformFromParentBodyNode(tf_trans);

    dart::dynamics::ShapeNode *shapeNodeCollision = terrainBodyCollision->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(terrainShape);

    shapeNodeCollision->setRelativeTransform(tf_trans);

    dart::dynamics::BodyNodePtr terrainBodyVisual
        = terrainSkel->createJointAndBodyNodePair<
                         dart::dynamics::WeldJoint>(nullptr).second;

    Eigen::Isometry3d tf_trans_vis = Eigen::Isometry3d::Identity();
    tf_trans_vis.translation() = Eigen::Vector3d{-config.xSize/2.0,config.ySize/2.0, 0.0};
    terrainBodyVisual->getParentJoint()->setTransformFromParentBodyNode(tf_trans_vis);

    dart::dynamics::ShapeNode *shapeNodeVisual = terrainBodyCollision->createShapeNodeWith<
        dart::dynamics::VisualAspect>(terrainShape);
    shapeNodeVisual->setRelativeTransform(tf_trans_vis);

    return  terrainSkel;
}

}



using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace std::chrono;

namespace DartRobots
{

class SomeWidget : public dart::gui::osg::ImGuiWidget
{
  public:
    /// Constructor
    SomeWidget(dart::gui::osg::ImGuiViewer *viewer, double *simTimeElapsed, double *realTimeElapsed)
        : viewer_(viewer), simTimeElapsed_(simTimeElapsed), realTimeElapsed_(realTimeElapsed){};

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-pro-type-vararg"
    // Documentation inherited
    void render() override
    {
        ImGui::SetNextWindowPos(ImVec2(10, 20));
        ImGui::SetNextWindowSize(ImVec2(420, 150));
        ImGui::SetNextWindowBgAlpha(0.5f);
        if (!ImGui::Begin("Information", nullptr, ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar))
        {
            // Early out if the window is collapsed, as an optimization.
            ImGui::End();
            return;
        }

        // Menu
        if (ImGui::BeginMenuBar())
        {
            if (ImGui::BeginMenu("Menu"))
            {
                if (ImGui::MenuItem("Exit"))
                    viewer_->setDone(true);
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Help"))
            {
                if (ImGui::MenuItem("About DART"))
                    viewer_->showAbout();
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        ImGui::Text("Mini Cheetah");

        ImGui::Text("Sim Time: %f", *simTimeElapsed_);
        ImGui::Text("Real Time: %f", *realTimeElapsed_);
        ImGui::Text("Rtf: %f", (*simTimeElapsed_) / (*realTimeElapsed_));
        ImGui::Spacing();

        ImGui::End();
    };
#pragma clang diagnostic pop

  private:
    dart::gui::osg::ImGuiViewer *viewer_;
    double *simTimeElapsed_, *realTimeElapsed_;
};

World::Impl::Impl()
{
    world_ = std::make_shared<dart::simulation::World>();
    world_->getConstraintSolver()->setCollisionDetector(dart::collision::OdeCollisionDetector::create());

    node_ = ::osg::ref_ptr<dart::gui::osg::WorldNode>(new dart::gui::osg::WorldNode(world_));
    node_->simulate(true);
}

void World::Impl::Step(uint64_t iter)
{
    if (duration_cast<microseconds>(startTime_.time_since_epoch()).count() == 0)
    {
        startTime_ = steady_clock::now();
    }
    for (unsigned i = 0; i < iter; i++)
    {

        node_->refresh();
        if (robot_ != nullptr)
        {
            robot_->SetContactDirty();
        }
    }
    auto endTime = steady_clock::now();
    realTimeElapsed_ = static_cast<double>(duration_cast<microseconds>(endTime - startTime_).count()) * 1e-6;
    simTimeElapsed_ = world_->getTime();
}

void World::Impl::Reset()
{
    if (robot_ != nullptr)
        robot_->Reset();
    simTimeElapsed_ = 0.0;
    realTimeElapsed_ = 0.0;
    world_->reset();
    startTime_ = steady_clock::now();
}

void World::Impl::Render()
{
    if (!viewer_)
    {
        viewer_ = std::make_unique<dart::gui::osg::ImGuiViewer>();
        viewer_->addWorldNode(node_);
        ::osg::ref_ptr<dart::gui::osg::GridVisual> gridVisual = new dart::gui::osg::GridVisual();
        gridVisual->setPlaneType(dart::gui::osg::GridVisual::PlaneType::XY);
        gridVisual->setOffset(Eigen::Vector3d(0, 0, 0.0));
        gridVisual->setNumCells(100);
        viewer_->addAttachment(gridVisual);

        // Add ImGui widget
        auto widget = std::make_shared<SomeWidget>(viewer_.get(), &simTimeElapsed_, &realTimeElapsed_);
        viewer_->getImGuiHandler()->addWidget(std::move(widget));

        viewer_->setUpViewInWindow(0, 0, 960, 640);

        viewer_->getCameraManipulator()->setHomePosition(
            ::osg::Vec3(2.57F, 3.14f, 1.64f), ::osg::Vec3(0.00f, 0.00f, 0.00f), ::osg::Vec3(-0.24f, -0.25f, 0.94f));
        //        // We need to re-dirty the CameraManipulator by passing it into the viewer
        //        // again, so that the viewer knows to update its HomePosition setting
        viewer_->setCameraManipulator(viewer_->getCameraManipulator());
    }

    node_->simulate(false);
    viewer_->frame();
    node_->simulate(true);
}

void World::Impl::SetRobot(std::shared_ptr<MiniCheetah> robot)
{
    robot_ = std::move(robot);
    robot_->SetWorld(world_);
    world_->addSkeleton(robot_->GetSkeleton());
}

void World::Impl::SetTerrain(Terrain terrain)
{
    if (terrain_ != nullptr)
    {
        world_->removeSkeleton(terrain_);
    }
    terrain_ = DartTerrainFromData(terrain);
    world_->addSkeleton(terrain_);
}

std::string World::Impl::AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                                 const std::string &name)
{
    auto tf = Eigen::Isometry3d::Identity();
    tf.translate(translation);
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(Frame::World(), name, tf);
    ShapePtr ball(new SphereShape(radius));
    frame->setShape(ball);
    frame->getVisualAspect(true)->setColor(color);
    return world_->addSimpleFrame(frame);
}
bool World::Impl::SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation)
{
    auto simpleFrame = world_->getSimpleFrame(name);
    if (simpleFrame == nullptr)
        return false;
    simpleFrame->setTranslation(translation);
    return true;
}
void World::Impl::DeleteBall(const std::string &name)
{
    auto frame = world_->getSimpleFrame(name);
    if (frame == nullptr)
        return;
    world_->removeSimpleFrame(frame);
}
std::string World::Impl::ChangeTerrain()
{
    return terrain_->getName();
}
void World::Impl::SetTerrainUrdf(const std::string &urdfPath)
{
    auto groundUri = dart::common::Uri();
    if (urdfPath.empty())
    {
        const std::string groundUrdfPath = Config::install_terrain_path + std::string("ground.urdf");
        groundUri.fromPath(groundUrdfPath);
    }
    else
    {
        groundUri.fromPath(urdfPath);
    }
    spdlog::debug("ground uri path: {}", groundUri.getPath());
    DartLoader urdfLoader;
    terrain_ = urdfLoader.parseSkeleton(groundUri);
    terrain_->disableSelfCollisionCheck();
    auto groundBodyNodes = terrain_->getBodyNodes();
    for (auto &bodyNode : groundBodyNodes)
    {
        // Set some reasonably high friction coeff here such that we will always be limited by foot friction
        // Allows for easier friction setting because we only set the foot friction
        bodyNode->setFrictionCoeff(100);
    }
    world_->addSkeleton(terrain_);
}
} // namespace DartRobots
