
from DartRobotsPy import Terrain
from DartRobotsPy import TerrainConfig
from DartRobotsPy import  TerrainType
from DartRobotsPy import  TerrainGenerator
from DartRobotsPy import get_height



# Configure the terrain

configPlane = TerrainConfig()
configHills = TerrainConfig()
configSteps = TerrainConfig()

#plane
configPlane.terrain_type = TerrainType.Plane
configPlane.x_size = 4.0
configPlane.y_size = 4.0
configPlane.resolution = 0.1


#Hills
configHills.terrain_type = TerrainType.Hills
configHills.x_size = 4.0
configHills.y_size = 4.0
configHills.resolution = 0.1
configHills.seed = 566
configHills.roughness = 0.001
configHills.amplitude = 0.2
configHills.frequency = 0.2
configHills.num_octaves = 1


#Steps
configSteps.terrain_type = TerrainType.Steps
configSteps.x_size = 4.0
configSteps.y_size = 4.0
configSteps.resolution = 0.1
configSteps.step_width = 0.2
configSteps.step_height = 0.1

# Generate the terrain
generator = TerrainGenerator()
print("\n")
print("Plane ")
terrain = generator.generate(configPlane)
print("Num vertices", len(terrain.heights))
print("height at 1.3333, 1.7777 : ", get_height(1.3333, 1.7777, terrain))
print("\n")

generator = TerrainGenerator()
print("Hills")
terrain = generator.generate(configHills)
print("Num vertices", len(terrain.heights))
print("height at 1.3333, 1.7777 : ", get_height(1.3333, 1.7777, terrain))
print("\n")

generator = TerrainGenerator()
print("Steps")
terrain = generator.generate(configSteps)
print("Num vertices", len(terrain.heights))
print("height at 1.3333, 1.7777 : ", get_height(1.3333, 1.7777, terrain))
print("\n")
