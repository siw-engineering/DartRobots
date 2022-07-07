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

Terrain TerrainGenerator::generateHills(const TerrainConfig &config)
{
    // We need (nx + 1) * (ny+1) vertices for a grid of size (nx * ny)
    size_t numVerticesX = static_cast<uint64_t>(config.xSize / config.resolution) + 1;
    size_t numVerticesY = static_cast<uint64_t>(config.ySize / config.resolution) + 1;

    PerlinNoise noiseGenerator;
    std::default_random_engine rd;
    auto randomGen = std::mt19937(rd());

    if (config.seed != -1)
    {
        noiseGenerator = PerlinNoise(config.seed);
        rd = std::default_random_engine(config.seed);
        randomGen = std::mt19937(rd());
    }

    Terrain terrain;
    std::vector<float> heights;
    double height{0};
    double r=0;
    for (uint64_t i = 0; i < numVerticesX; ++i)
    {
       r = 0;
        for (uint64_t j = 0; j < numVerticesY; ++j)
        {

            height = 0;
            // generate perlin noise based terrain
            for (int k = 0; k < config.numOctaves; ++k)
            {
                double amp = config.amplitude / pow(2, k);
                double freq = config.frequency * pow(2, k);

                height += amp * noiseGenerator.noise(i * freq, j * freq, 0.1);
            }

            // TODO : add roughness here
            height += config.roughness * uniformDist_(randomGen);
            if (config.slope != 0)
            {
                auto h = (-config.xSize / 2) + r * sin(config.slope * (3.1415 / 180));
                r += config.resolution;
                heights.emplace_back(height + h + config.xSize / 2);
            }
            else
            {
                heights.emplace_back(height);
            }
        }
    }

    if(config.slopeX)
    {
        Eigen::Map<Eigen::Matrix<float, -1, -1>> hmap(heights.data(), static_cast<int64_t>(numVerticesX),
                                                      static_cast<int64_t>(numVerticesY));
        hmap.transpose();

        for (auto i = 0; i < numVerticesX; i++)
        {
            for (auto j = 0; j < numVerticesY; j++)
            {
                terrain.heights.emplace_back(hmap.row(i)(j));
            }
        }
    }
    else
    {
        terrain.heights = heights;
    }
    terrain.config = config;

    return terrain;
}

Terrain TerrainGenerator::generatePlane(const TerrainConfig &config)
{
    // We need (nx + 1) * (ny+1) vertices for a grid of size (nx * ny)
    size_t numVerticesX = static_cast<uint64_t>(config.xSize / config.resolution) + 1;
    size_t numVerticesY = static_cast<uint64_t>(config.ySize / config.resolution) + 1;

    Terrain terrain;
    std::vector<float> heights;
    double r = 0;
    for(uint64_t i =0 ; i < numVerticesX; i++)
    {   r = 0;
        for(uint64_t j = 0; j < numVerticesY; j++)
        {
            auto h = (-config.xSize/2) + r * sin(config.slope * (3.1415 / 180));
            heights.emplace_back(h + config.xSize/2);
            r += config.resolution;
        }
    }

    if(config.slopeX)
    {
        Eigen::Map<Eigen::Matrix<float, -1, -1>> hmap(heights.data(), static_cast<int64_t>(numVerticesX),
                                                      static_cast<int64_t>(numVerticesY));
        hmap.transpose();

        for (auto i = 0; i < numVerticesX; i++)
        {
            for (auto j = 0; j < numVerticesY; j++)
            {
                terrain.heights.emplace_back(hmap.row(i)(j));
            }
        }
    }
    else
    {
        terrain.heights = heights;
    }

    terrain.config = config;

    return terrain;
}

Terrain TerrainGenerator::generateSteps(const TerrainConfig &config)
{
    // We need (nx + 1) * (ny+1) vertices for a grid of size (nx * ny)
    const size_t nVerticesX = static_cast<size_t>(config.xSize / config.resolution) + 1;
    const size_t nVerticesY = static_cast<size_t>(config.ySize / config.resolution) + 1;

    // Calculate the no: of segments of `stepWidth` that will fit in the terrain size
    int nSegmentsX = static_cast<int32_t>(config.xSize / config.stepWidth);
    int nSegmentsY = static_cast<int32_t>(config.ySize / config.stepWidth);

    // calculate number of vertices needed for each segment
    int VerticesPerSegment = static_cast<int32_t>(config.stepWidth / config.resolution);

    // Using eigen to avoid explicit looping to set heights in a std::vector<>
    // The values for full square can be set using block
    std::vector<float> heights;
    heights.resize(nVerticesX * nVerticesY);
    // Using map instead of Matrix eliminates the need to convert from eigen matrix
    // to std::vector as Map uses in place operations
    Eigen::Map<Eigen::Matrix<float, -1, -1>> hmap(heights.data(), static_cast<int64_t>(nVerticesX),
                                                  static_cast<int64_t>(nVerticesY));
    hmap.setZero();

    std::default_random_engine rd;
    auto randomGen = std::mt19937(rd());

    if (config.seed != -1)
    {
        rd = std::default_random_engine(config.seed);
        randomGen = std::mt19937(rd());
    }

    // Loop through the squares with (VerticesPerSegment * VerticesPerSegment) vertices each
    double height; // NOLINT(cppcoreguidelines-init-variables)
    for (int64_t i = 0; i < nSegmentsX; ++i)
    {
        for (int64_t j = 0; j < nSegmentsY; ++j)
        {
            // set random height in range (0, 0.5)
            height = uniformDist_(randomGen) * config.stepHeight;
            hmap.block(i * VerticesPerSegment, j * VerticesPerSegment, VerticesPerSegment, VerticesPerSegment)
                .setConstant(static_cast<float>(height));
        }
    }

    Terrain terrain;
    terrain.heights = heights;
    terrain.config = config;

    return terrain;
}
