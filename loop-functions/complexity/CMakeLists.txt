# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

#add_library(aggregation_xor SHARED AggregationTwoSpotsXOR.h AggregationTwoSpotsXOR.cpp)
#target_link_libraries(aggregation_xor argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(aggregation_xor2 SHARED AggregationTwoSpotsXOR2.h AggregationTwoSpotsXOR2.cpp)
target_link_libraries(aggregation_xor2 argos3plugin_${ARGOS_BUILD_FOR}_epuck)

#add_library(aggregation_5050 SHARED AggregationTwoSpots5050.h AggregationTwoSpots5050.cpp)
#target_link_libraries(aggregation_5050 argos3plugin_${ARGOS_BUILD_FOR}_epuck)

#add_library(aggregation_single SHARED AggregationSingleSpot.h AggregationSingleSpot.cpp)
#target_link_libraries(aggregation_single argos3plugin_${ARGOS_BUILD_FOR}_epuck)

#add_library(empty_arena SHARED EmptyArena.h EmptyArena.cpp)
#target_link_libraries(empty_arena argos3plugin_${ARGOS_BUILD_FOR}_epuck)

#add_library(two_spots_arena SHARED AggregationTwoSpots.h AggregationTwoSpots.cpp)
#target_link_libraries(two_spots_arena argos3plugin_${ARGOS_BUILD_FOR}_epuck)
