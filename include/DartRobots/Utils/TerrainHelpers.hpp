#ifndef DARTROBOTS_TERRAINHELPERS_HPP
#define DARTROBOTS_TERRAINHELPERS_HPP
#include "TerrainGenerator.hpp"
#include <dart/dart.hpp>

namespace Terrains
{
dart::dynamics::SkeletonPtr DartTerrainFromData(Terrain terrain);
double GetHeight(double x, double y, Terrain &terrain);
} // namespace Terrains

#endif // DARTROBOTS_TERRAINHELPERS_HPP
