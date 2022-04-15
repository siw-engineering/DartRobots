#include "DartRobots/MiniCheetah.hpp"
#include "DartRobots/World.hpp"
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>
using namespace std::chrono;

int main()
{
    spdlog::set_level(spdlog::level::trace);
    auto world = DartRobots::World();
    auto robot = std::make_shared<DartRobots::MiniCheetah>(
        DartRobots::MiniCheetahConfig{.spawnPos = Eigen::Vector3d(0.5, 0.0, 0.5)});
    world.SetTerrainUrdf();
    world.SetRobot(robot);
    auto startTime = steady_clock::now();
    auto markerName = world.AddBall(Eigen::Vector3d(0.19, 0.11, -0.29), Eigen::Vector3d(0.9, 0.0, 0.0), 0.05);
    auto markerName2 = world.AddBall(Eigen::Vector3d(0.19, -0.11, -0.29), Eigen::Vector3d(0.9, 0.0, 0.0), 0.05);
    std::cout << "Add ball with marker name: " << markerName << std::endl;
    std::cout << "Add ball with marker name: " << markerName2 << std::endl;
    robot->SaveState(0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 250; j++)
        {
            robot->SetJointCommands(Eigen::Matrix<double, 12, 1>::Zero());
            world.Step(1);
            auto footPos = robot->GetFootPositions();
            world.SetBallTranslation(markerName, footPos.block<3, 1>(0, 0));
            world.SetBallTranslation(markerName2, footPos.block<3, 1>(0, 1));
            world.Render();
        }
        robot->LoadState(0);
        world.Reset();
        world.DeleteBall(markerName2);
    }
    auto endTime = steady_clock::now();
    auto timeDiffMs = duration_cast<milliseconds>(endTime - startTime).count();
    std::cout << "Time taken: " << timeDiffMs << std::endl;
}