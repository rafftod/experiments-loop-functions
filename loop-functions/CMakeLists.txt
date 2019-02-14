# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
link_directories(
	${CMAKE_BINARY_DIR}/src
    )

add_subdirectory(chocolate)
#add_subdirectory(example)
#add_subdirectory(gianduja)
add_subdirectory(complexity)
add_subdirectory(vanilla)
#add_subdirectory(extra)
#add_subdirectory(NEATvsEvo)
#add_subdirectory(visual-add-ons)
#add_subdirectory(debug)
