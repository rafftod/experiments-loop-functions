add_library(shelter_fifty_fifty_loopfunc SHARED ShelterFiftyFiftyLoopFunc.h ShelterFiftyFiftyLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(shelter_fifty_fifty_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)


add_library(sca_loopfunc SHARED
                          ShelterConstrainedAccessLoopFunc.h
                          ShelterConstrainedAccessLoopFunc.cpp
                          SCAbehindLoopFunc.h
                          SCAbehindLoopFunc.cpp
                          SCAblackLoopFunc.h
                          SCAblackLoopFunc.cpp
                          SCAaroundLoopFunc.h
                          SCAaroundLoopFunc.cpp
                          AggregationSingleSpot.h
                          AggregationSingleSpot.cpp
                          DirectionalGate.h
                          DirectionalGate.cpp
                          ${LOOP_HEADERS}
                          ${LOOP_SOURCES})
target_link_libraries(sca_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
