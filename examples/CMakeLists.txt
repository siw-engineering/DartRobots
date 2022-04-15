add_executable(MiniCheetahExample MiniCheetah.cpp)
target_link_libraries(MiniCheetahExample PRIVATE MiniCheetah)


install(
        TARGETS MiniCheetahExample
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)