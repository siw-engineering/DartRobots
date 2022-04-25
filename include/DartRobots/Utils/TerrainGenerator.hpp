#ifndef TERRAINGENERATOR_TERRAINGENERATOR_H
#define TERRAINGENERATOR_TERRAINGENERATOR_H

#include "PerlinNoise.hpp"
#include <random>
#include <vector>

enum struct TerrainType
{
    Invalid = 0,
    Hills,
    Steps,
    Plane
};

struct TerrainConfig
{
    TerrainType terrainType = TerrainType::Invalid;
    int seed = -1;

    float xSize, ySize = -1; // in meters
    float resolution = -1;   // size of a square in meters

    // To be used with hill terrain
    float roughenss = -1;
    float amplitude = -1;
    float frequency = -1;
    int numOctaves = -1;

    // To be used with steps and stairs
    float stepWidth = -1;
    float stepHeight = -1;
};

struct Terrain
{
    std::vector<float> heights;
    TerrainConfig config;
};
namespace Terrains
{
class TerrainGenerator
{

  public:
    TerrainGenerator();

    // Returns an instance of Terrain based on provided configuration
    Terrain generate(const TerrainConfig &config);

    Terrain generateHills(const TerrainConfig &config);
    Terrain generateSteps(const TerrainConfig &config);
    Terrain generatePlane(const TerrainConfig &config);

  protected:
    std::uniform_real_distribution<float> uniformDist_;
};
}
#endif //TERRAINGENERATOR_TERRAINGENERATOR_H