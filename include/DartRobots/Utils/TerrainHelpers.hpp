#ifndef DARTROBOTS_TERRAINHELPERS_HPP
#define DARTROBOTS_TERRAINHELPERS_HPP
#include "TerrainGenerator.hpp"
#include <dart/dart.hpp>

namespace Terrains
{
dart::dynamics::SkeletonPtr DartTerrainFromData(Terrain terrain);
float GetHeight(float x,float y, Terrain& terrain);

}

#endif // DARTROBOTS_TERRAINHELPERS_HPP
