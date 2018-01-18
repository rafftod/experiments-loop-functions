
add_library(chocolate_cfa_loopfunc SHARED ChocolateCFALoopFunc.h ChocolateCFALoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
target_link_libraries(chocolate_cfa_loopfunc argos3plugin_${ARGOS_BUILD_FOR}_epuck)
