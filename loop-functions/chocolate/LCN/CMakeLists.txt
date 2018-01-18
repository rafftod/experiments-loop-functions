add_library(chocolate_lcn_loopfunc SHARED ChocolateLCNLoopFunc.h ChocolateLCNLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(chocolate_lcn_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
