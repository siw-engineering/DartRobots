#include "MiniCheetahImpl.hpp"
#include "DartRobots/Config.hpp"
#include "Helpers.hpp"
#include <chrono>
#include <dart/collision/ode/ode.hpp>
#include <dart/constraint/constraint.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/simulation/World.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <utility>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace std::chrono;
using namespace std::chrono_literals;

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

MiniCheetah::Impl::Impl(MiniCheetahConfig config) : config_(std::move(config))
{
    const std::string robotName = "mini_cheetah";
    auto groundUri = dart::common::Uri();
    auto robotUri = dart::common::Uri();

    auto get_exec_path = []() -> std::string {
        std::array<char, PATH_MAX> buff;
        ssize_t len = ::readlink("/proc/self/exe", buff.data(), sizeof(buff) - 1);
        if (len != -1)
        {
            buff[len] = '\0';
            return {buff.data()};
        }
        return "";
    };

    auto exec_path = get_exec_path();
    const std::string robotUrdfPath = fmt::format("{}{}/{}.urdf", Config::install_robots_path, robotName, robotName);
    const std::string groundUrdfPath = Config::install_terrain_path + std::string("ground.urdf");
    auto robotAbsPath = std::filesystem::absolute(robotUrdfPath);
    auto groundAbsPath = std::filesystem::absolute(Config::install_terrain_path + std::string("ground.urdf"));
    groundUri.fromRelativeUri(exec_path.c_str(), groundUrdfPath.c_str());
    robotUri.fromRelativeUri(exec_path.c_str(), robotUrdfPath.c_str());
    spdlog::debug("ground uri path: {}", groundUri.getPath());
    spdlog::debug("robot uri path: {}", robotUri.getPath());

    DartLoader urdfLoader;
    ground_ = urdfLoader.parseSkeleton(groundUri);
    ground_->disableSelfCollisionCheck();
    auto groundBodyNodes = ground_->getBodyNodes();
    for (auto &bodyNode : groundBodyNodes)
    {
        // Set some reasonably high friction coeff here such that we will always be limited by feet friction
        // Allows for easier friction setting because we only set the feet friction
        bodyNode->setFrictionCoeff(100);
    }
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

    world_ = std::make_shared<dart::simulation::World>();
    world_->getConstraintSolver()->setCollisionDetector(dart::collision::OdeCollisionDetector::create());
    world_->addSkeleton(ground_);
    world_->addSkeleton(robot_);

    node_ = ::osg::ref_ptr<dart::gui::osg::WorldNode>(new dart::gui::osg::WorldNode(world_));
    node_->simulate(true);
}
void MiniCheetah::Impl::SetJointCommands(Eigen::Matrix<double, 12, 1> commands)
{
    for (unsigned i = 0; i < 12; i++)
    {
        revoluteJoints_.at(i)->setCommand(0, commands(i));
    }
}

void MiniCheetah::Impl::Step(uint64_t iter)
{
    if (duration_cast<microseconds>(startTime_.time_since_epoch()).count() == 0)
    {
        startTime_ = steady_clock::now();
    }
    for (unsigned i = 0; i < iter; i++)
    {

        node_->refresh();
        contactDataDirty_ = true;
    }
    auto endTime = steady_clock::now();
    realTimeElapsed_ = static_cast<double>(duration_cast<microseconds>(endTime - startTime_).count()) * 1e-6;
    simTimeElapsed_ = world_->getTime();
}

void MiniCheetah::Impl::Render()
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

void MiniCheetah::Impl::Reset()
{
    coulombFriction_ = Eigen::Matrix<double, 12, 1>::Zero();
    viscousFriction_ = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 4; i++)
    {
        footFriction_(i) = legNodes_.at(i)->getFrictionCoeff();
    }
    simTimeElapsed_ = 0.0;
    realTimeElapsed_ = 0.0;
    world_->reset();
    startTime_ = steady_clock::now();
    contactDataDirty_ = true;
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
    auto footContactStates = Eigen::Matrix<bool, 4, 1>{};
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

std::string MiniCheetah::Impl::AddBall(const Eigen::Vector3d &translation, const Eigen::Vector3d &color, double radius,
                                       const std::string &name)
{
    auto tf = Eigen::Isometry3d::Identity();
    tf.translate(translation);
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(robot_->getBodyNode(0), name, tf);
    ShapePtr ball(new SphereShape(radius));
    frame->setShape(ball);
    frame->getVisualAspect(true)->setColor(color);
    return world_->addSimpleFrame(frame);
}
bool MiniCheetah::Impl::SetBallTranslation(const std::string &name, const Eigen::Vector3d &translation,
                                           const std::string &frame)
{
    auto simpleFrame = world_->getSimpleFrame(name);
    if (simpleFrame == nullptr)
        return false;
    Frame *translationFrame;
    if (frame.empty())
    {
        translationFrame = Frame::World();
    }
    else
    {
        translationFrame = robot_->getBodyNode(frame);
    }
    if (translationFrame == nullptr)
    {
        spdlog::info("SetBallTranslation failed, maybe frame name does not name a robot link");
        return false;
    }
    simpleFrame->setTranslation(translation, translationFrame);
    return true;
}
void MiniCheetah::Impl::DeleteBall(const std::string &name)
{
    auto frame = world_->getSimpleFrame(name);
    if (frame == nullptr)
        return;
    world_->removeSimpleFrame(frame);
}
} // namespace DartRobots