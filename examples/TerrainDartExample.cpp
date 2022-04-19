
#include "DartRobots/MiniCheetah.hpp"
#include "DartRobots/World.hpp"
#include "TerrainGenerator.h"
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>

using namespace std::chrono;

int main()
{
    spdlog::set_level(spdlog::level::trace);
    auto world = DartRobots::World();
    auto startTime = steady_clock::now();
    TerrainGenerator generator;
    TerrainConfig config;

    config.terrainType = TerrainType::Steps;
    config.xSize = config.ySize = 4.0;
    config.resolution = 0.02;
    // Hills
    config.amplitude = 0.2;
    config.frequency = 0.2;
    config.roughenss = 0.000;
    config.numOctaves = 1;


    // Steps
    config.stepWidth = 0.2;
    config.stepHeight = 0.1;


    auto terrain = generator.generate(config);


    auto robot = std::make_shared<DartRobots::MiniCheetah>(
        DartRobots::MiniCheetahConfig{.spawnPos = Eigen::Vector3d(0.5, 0.0, 0.5)});

    world.SetRobot(robot);
    world.SetTerrain(terrain);

    robot->SaveState(0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 250; j++)
        {
            robot->SetJointCommands(Eigen::Matrix<double, 12, 1>::Zero());
            world.Step(1);
            auto footPos = robot->GetFootPositions();
            world.Render();
        }
        robot->LoadState(0);
        world.Reset();
    }

    auto endTime = steady_clock::now();
    auto timeDiffMs = duration_cast<milliseconds>(endTime - startTime).count();
    std::cout << "Time taken: " << timeDiffMs << std::endl;

}