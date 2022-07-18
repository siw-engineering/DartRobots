#include "TerrainGenerator.hpp"
#include <Eigen/Dense>
#include <math.h>


using namespace  Eigen;
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
    Terrain terrain;

    int nVerticesX = static_cast<uint64_t>(config.xSize / config.resolution) + 1;
    int nVerticesY = static_cast<uint64_t>(config.ySize / config.resolution) + 1;

    MatrixXf heights;
    heights.resize(nVerticesX, nVerticesY);
    heights.setZero();

    if (config.slope != 0)
    {
        VectorXf offsets;
        offsets.resize(nVerticesX);
        offsets.setZero();

        for(int k = 0; k < nVerticesX; k++)
            offsets(k) = ((config.resolution* k) * sin(config.slope * (3.1415 / 180)));

        if (!config.slopeX)
            heights.rowwise() += offsets.transpose();
        else
            heights.colwise() += offsets;
    }

    // Convert into std::vector<float>
    int r, c = 0;
    for(int i = 0; i < nVerticesX * nVerticesY; i++)
    {
        r = i / nVerticesX;
        c = i % nVerticesY;
        terrain.heights.emplace_back(heights.row(r)[c]);
    }

    terrain.config = config;
    return terrain;
}

Terrain TerrainGenerator::generateSteps(const TerrainConfig &config)
{
    Terrain terrain;

    const int nVerticesX = static_cast<size_t>(config.xSize / config.resolution) + 1;
    const int nVerticesY = static_cast<size_t>(config.ySize / config.resolution) + 1;

    int nSegmentsX = static_cast<int32_t>(config.xSize / config.stepWidth);
    int nSegmentsY = static_cast<int32_t>(config.ySize / config.stepWidth);

    int VerticesPerSegment = static_cast<int32_t>(config.stepWidth / config.resolution);

    MatrixXf heights;
    heights.resize(nVerticesX, nVerticesY);
    heights.setZero();

    std::default_random_engine rd;
    auto randomGen = std::mt19937(rd());

    if (config.seed != -1)
    {
        rd = std::default_random_engine(config.seed);
        randomGen = std::mt19937(rd());
    }

    double height = 0;
    for (int64_t i = 0; i < nSegmentsX; ++i)
    {
        for (int64_t j = 0; j < nSegmentsY; ++j)
        {
            // set random height in range (0, 0.5)
            height = uniformDist_(randomGen) * config.stepHeight;
            heights.block(i * VerticesPerSegment, j * VerticesPerSegment, VerticesPerSegment, VerticesPerSegment)
                .setConstant(static_cast<float>(height));
        }
    }

    if (config.slope != 0)
    {
        VectorXf offsets;
        offsets.resize(nVerticesX);
        offsets.setZero();

        for(int k = 0; k < nVerticesX; k++)
            offsets(k) = ((config.resolution* k) * sin(config.slope * (3.1415 / 180)));

        if (!config.slopeX)
            heights.rowwise() += offsets.transpose();
        else
            heights.colwise() += offsets;
    }

    // Convert into std::vector<float>
    int r, c = 0;
    for(int i = 0; i < nVerticesX * nVerticesY; i++)
    {
        r = i / nVerticesX;
        c = i % nVerticesY;
        terrain.heights.emplace_back(heights.row(r)[c]);
    }

    terrain.config = config;
    return terrain;

}

Terrain TerrainGenerator::generateHills(const TerrainConfig &config)
{
    Terrain terrain;

    int nVerticesX = static_cast<uint64_t>(config.xSize / config.resolution) + 1;
    int nVerticesY = static_cast<uint64_t>(config.ySize / config.resolution) + 1;

    MatrixXf heights;
    heights.resize(nVerticesX, nVerticesY);
    heights.setZero();

    PerlinNoise noiseGenerator;
    std::default_random_engine rd;
    auto randomGen = std::mt19937(rd());

    if (config.seed != -1)
    {
        noiseGenerator = PerlinNoise(config.seed);
        rd = std::default_random_engine(config.seed);
        randomGen = std::mt19937(rd());
    }

    int i,j = 0;
    for(int idx = 0; idx < nVerticesY * nVerticesY; idx++)
    {
        i = idx / nVerticesX;
        j = idx % nVerticesY;
        double amp, freq = 0;
        float height = 0;

        for(int nOctave = 0; nOctave < config.numOctaves; nOctave++)
        {
            amp = config.amplitude / pow(2, nOctave);
            freq = config.frequency * pow(2, nOctave);

            height += amp * noiseGenerator.noise(i * freq, j * freq, 0.1);
        }
        height += config.roughness * uniformDist_(randomGen);

        heights.row(i)[j] = height;
    }

    if (config.slope != 0)
    {
        VectorXf offsets;
        offsets.resize(nVerticesX);
        offsets.setZero();

        for(int k = 0; k < nVerticesX; k++)
            offsets(k) = ((config.resolution* k) * sin(config.slope * (3.1415 / 180)));

        if (!config.slopeX)
            heights.rowwise() += offsets.transpose();
        else
            heights.colwise() += offsets;
    }

    // Convert into std::vector<float>
    int r, c = 0;
    for(int idx = 0; idx < nVerticesX * nVerticesY; idx++)
    {
        r = idx / nVerticesX;
        c = idx % nVerticesY;
        terrain.heights.emplace_back(heights.row(r)[c]);
    }

    terrain.config = config;
    return terrain;

}