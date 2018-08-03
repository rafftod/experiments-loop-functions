# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_library(aggregation_loopfunc SHARED AggregationTwoSpotsLoopFunc.h AggregationTwoSpotsLoopFunc.cpp)
target_link_libraries(aggregation_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(foraging_loopfunc SHARED ForagingTwoSpotsLoopFunc.h ForagingTwoSpotsLoopFunc.cpp)
target_link_libraries(foraging_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)


