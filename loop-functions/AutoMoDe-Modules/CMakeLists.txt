# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(stop_loopfunc SHARED StopLoopFunc.h StopLoopFunc.cpp)
target_link_libraries(stop_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(exploration_loopfunc SHARED ExplorationLoopFunc.h ExplorationLoopFunc.cpp)
target_link_libraries(exploration_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(repulsion_loopfunc SHARED RepulsionLoopFunc.h RepulsionLoopFunc.cpp)
target_link_libraries(repulsion_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(attraction_loopfunc SHARED AttractionLoopFunc.h AttractionLoopFunc.cpp)
target_link_libraries(attraction_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(phototaxis_loopfunc SHARED PhototaxisLoopFunc.h PhototaxisLoopFunc.cpp)
target_link_libraries(phototaxis_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(antiphototaxis_loopfunc SHARED AntiPhototaxisLoopFunc.h AntiPhototaxisLoopFunc.cpp)
target_link_libraries(antiphototaxis_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
