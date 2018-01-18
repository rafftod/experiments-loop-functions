# Set up ARGoS compilation information
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})

add_library(gianduja_aggregation_loopfunc SHARED GiandujaAggregationLoopFunc.h GiandujaAggregationLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_library(gianduja_aggregation_2spots_loopfunc SHARED GiandujaAggregation2SpotsLoopFunc.h GiandujaAggregation2SpotsLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_library(gianduja_nesting_loopfunc SHARED GiandujaNestingLoopFunc.h GiandujaNestingLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_library(gianduja_nesting_hide_loopfunc SHARED GiandujaNestingHideLoopFunc.h GiandujaNestingHideLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_library(gianduja_decision_loopfunc SHARED GiandujaDecisionLoopFunc.h GiandujaDecisionLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_library(gianduja_stop_loopfunc SHARED GiandujaStopLoopFunc.h GiandujaStopLoopFunc.cpp ${LOOP_HEADERS} ${LOOP_SOURCES})
add_subdirectory(example)
add_subdirectory(chocolate)
