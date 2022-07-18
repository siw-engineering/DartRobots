#include "TerrainGenerator.hpp"
#include <Eigen/Dense>
#include <math.h>

using namespace Terrains;

TerrainGenerator::TerrainGenerator()
{

    uniformDist_ = std::uniform_real_distribution<float>(0.0, 1.0);
}

Terrain TerrainGenerator::generate(const TerrainConfig &config)
{

    switch (config.terrainType)
    {
    case TerrainType::Hills:
        return generateHills(config);
    case TerrainType::Steps:
        return generateSteps(config);
    case TerrainType::Plane:
        return generatePlane(config);
    default:
        return Terrain();
    }
}

Terrain TerrainGenerator::generatePlane(const TerrainConfig &config)
{

    using namespace  Eigen;
    Terrain terrain;

    size_t numVerticesX = static_cast<uint64_t>(config.xSize / config.resolution) + 1;
    size_t numVerticesY = static_cast<uint64_t>(config.ySize / config.resolution) + 1;

    VectorXd offsets;
    offsets.resize(numVerticesX);
    offsets.setZero();

    MatrixXd heights;
    heights.resize(numVerticesX, numVerticesY);

    if (config.slope != 0)
    {
        for(int k = 0; k < numVerticesX; k++)
            offsets(k) = ((config.resolution* k) * sin(config.slope * (3.1415 / 180)));

        // Scoping for readability
        {
            if (config.slopeX)
                for (int k = 0; k < numVerticesX; k++)
                    heights.col(k) = offsets;
            else
                for (int k = 0; k < numVerticesX; k++)
                    heights.row(k) = offsets;
        }
    }


    // Convert into std::vector<float>
    int r, c = 0;
    for(int i = 0; i < numVerticesX * numVerticesY; i++)
    {
        r = i / numVerticesX;
        c = i % numVerticesY;

        terrain.heights.emplace_back(heights.row(r)[c]);

    }

    terrain.config = config;
    return terrain;
}