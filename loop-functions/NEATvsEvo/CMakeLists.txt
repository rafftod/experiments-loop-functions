add_library(shelter_fifty_fifty_loopfunc SHARED ShelterFiftyFiftyLoopFunc.h ShelterFiftyFiftyLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(shelter_fifty_fifty_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(shelter_constrained_access_loopfunc SHARED ShelterConstrainedAccessLoopFunc.h ShelterConstrainedAccessLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(shelter_constrained_access_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(sca_behind_loopfunc SHARED SCAbehindLoopFunc.h SCAbehindLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(sca_behind_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)

add_library(sca_black_loopfunc SHARED SCAblackLoopFunc.h SCAblackLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(sca_black_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
