#include "DartRobots/MiniCheetah.hpp"
#include "DartRobots/World.hpp"
#include "TerrainGenerator.hpp"
#include "TerrainHelpers.hpp"
#include <chrono>
#include <iostream>
#include <spdlog/spdlog.h>

using namespace std::chrono;
using namespace  Terrains;

int main()
{
    spdlog::set_level(spdlog::level::trace);
    auto world = DartRobots::World();
    auto startTime = steady_clock::now();
    TerrainGenerator generator;
    TerrainConfig config;

    config.seed = 633;
    config.terrainType = TerrainType::Steps;
    config.xSize = config.ySize = 4.0;
    config.resolution = 0.01;
    // Hills
    config.amplitude = 0.2;
    config.frequency = 0.2;
    config.roughenss = 0.000;
    config.numOctaves = 1;


    // Steps
    config.stepWidth = 0.2;
    config.stepHeight = 0.1;


    auto terrain = generator.generate(config);

    world.SetTerrain(terrain);

    auto xs = (terrain.config.xSize /terrain.config.resolution) + 1;
    auto ys = (terrain.config.ySize /terrain.config.resolution) + 1;
    Eigen::Map<Eigen::Matrix<float,  -1, -1>> hmap(terrain.heights.data(),
                                                  xs,
                                                  ys);
    auto xi = 1.0;
    auto yi = 1.0;

    auto dx = 0.222222;
    auto dy = 0.111111;
    float  h = GetHeight(xi + dx,yi + dy, terrain);
    std::cout << "Computed height at ("<<xi + dx <<" , "<<yi + dy<< ") : "<<h<<std::endl;

    auto markerName = world.AddBall(Eigen::Vector3d(xi + dx, (yi + dy), h),
                                    Eigen::Vector3d(0.9, 0.0, 0.0), 0.05);

    for(int i = 0; i < 500; i++)
    {
        world.Step(1);
        if (i % 20)
        world.Render();
    }
    world.DeleteBall(markerName);
}