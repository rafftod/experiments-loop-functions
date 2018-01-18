# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

# Headers
set(LOOP_HEADERS
	core/CoreLoopFunctions.h)

# Sources
set(AUTOMODE_SOURCES
	core/CoreLoopFunctions.cpp
)
