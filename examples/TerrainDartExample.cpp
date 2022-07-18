#include "DartRobots/MiniCheetah.hpp"
#include "DartRobots/World.hpp"
#include "TerrainGenerator.hpp"
#include "TerrainHelpers.hpp"
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace Terrains;

int main()
{
    spdlog::set_level(spdlog::level::trace);
    auto world = DartRobots::World();
    TerrainGenerator generator;
    TerrainConfig config;

    config.seed = 633;
    config.terrainType = TerrainType::Hills;
    config.xSize = config.ySize = 4.0;
    config.resolution = 0.05;
    // Hills
    config.amplitude = 0.2;
    config.frequency = 0.2;
    config.roughness = 0.000;
    config.numOctaves = 1;

    // Steps
    config.stepWidth = 0.2;
    config.stepHeight = 0.1;

    config.slopeX = true;
    config.slope = 20; // degrees

    auto terrain = generator.generate(config);
    auto robot = std::make_shared<DartRobots::MiniCheetah>(
        DartRobots::MiniCheetahConfig{.spawnPos = Eigen::Vector3d(0.5, 0.0, 1.5)});
//    world.SetRobot(robot);

    robot->SaveState(0);
    std::vector<std::string> ballNames{};
    for (int i = 0; i < 3; i++)
    {
        // Change terrain
        if (i % 3 == 0)
            config.terrainType = TerrainType::Hills;
        else if (i % 3 == 1)
            config.terrainType = TerrainType::Steps;
        else if (i % 3 == 2)
            config.terrainType = TerrainType::Stairs;
        else
            config.terrainType = TerrainType::Plane;
        terrain = generator.generate(config);
        for (int j = 0; j < 300; j++)
        {
            const auto x = 0.112 + j * 0.006;
            const auto y = 0.0 + j * 0.005;
            const auto height = GetHeight(x, y, terrain);
            if (i == 0)
            {
                std::string tempName = fmt::format("marker_{}", j);
                const auto name =
                    world.AddBall(Eigen::Vector3d(x, y, height), Eigen::Vector3d(1.0, 0.0, 0.0), 0.005, tempName);
                ballNames.emplace_back(name);
            }
            else
            {
                const auto &ballName = ballNames.at(j);
                world.SetBallTranslation(ballName, Eigen::Vector3d(x, y, height));
            }
        }
        world.SetTerrain(terrain);

        for (int j = 0; j < 250; j++)
        {
            robot->SetJointCommands(Eigen::Matrix<double, 12, 1>::Zero());
            world.Step(1);
            auto footPos = robot->GetFootPositions();
            world.Render();
        }
        robot->LoadState(0);
        world.Reset();
        std::this_thread::sleep_for(1s);
    }

    world.Render();
}