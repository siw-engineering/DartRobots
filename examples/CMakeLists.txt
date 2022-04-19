add_executable(MiniCheetahExample MiniCheetah.cpp)
target_link_libraries(MiniCheetahExample PRIVATE MiniCheetah spdlog::spdlog)

#add_executable(TerrainGenerationExample TerrainGeneration.cpp)
#target_link_libraries(TerrainGenerationExample PRIVATE TerrainGenerator)

add_executable(DartTerrainExample TerrainDartExample.cpp)
target_link_libraries(DartTerrainExample PRIVATE TerrainGenerator
        MiniCheetah
        spdlog::spdlog)

install(
        TARGETS MiniCheetahExample
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)