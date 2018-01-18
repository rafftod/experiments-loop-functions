if(ARGOS_BUILD_FOR_SIMULATOR)
	# Set up ARGoS compilation information
	include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
	link_directories(${ARGOS_LIBRARY_DIRS})

	# Headers
	set(LOOP_HEADERS
		CoreLoopFunctions.h
	)

	# Sources
	set(LOOP_SOURCES
		CoreLoopFunctions.cpp
	)
else()
endif(ARGOS_BUILD_FOR_SIMULATOR)


add_library(argos3_demiurge_loop_functions SHARED ${LOOP_HEADERS} ${LOOP_SOURCES} )

install(
  FILES ${LOOP_HEADERS}
  DESTINATION include/argos3/demiurge/loop-functions)

# Install location for the e-puck plugin
install(TARGETS argos3_demiurge_loop_functions
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
