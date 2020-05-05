
include_directories( ../../../argos3-Automode/beacon )

add_library(gianduja_aggregation_loopfunc SHARED GiandujaAggregationLoopFunc.h GiandujaAggregationLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_aggregation_2spots_loopfunc SHARED GiandujaAggregation2SpotsLoopFunc.h GiandujaAggregation2SpotsLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_nesting_loopfunc SHARED GiandujaNestingLoopFunc.h GiandujaNestingLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_nesting_hide_loopfunc SHARED GiandujaNestingHideLoopFunc.h GiandujaNestingHideLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_decision_loopfunc SHARED GiandujaDecisionLoopFunc.h GiandujaDecisionLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_stop_loopfunc SHARED GiandujaStopLoopFunc.h GiandujaStopLoopFunc.cpp )
target_link_libraries(gianduja_aggregation_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_beacon_aggregation_loopfunc SHARED GiandujaBeaconAggregationLoopFunc.h GiandujaBeaconAggregationLoopFunc.cpp
../../../argos3-Automode/beacon/epuck_beacon.h  ../../../argos3-Automode/beacon/epuck_beacon.cpp
)
target_link_libraries(gianduja_beacon_aggregation_loopfunc argos3_demiurge_loop_functions )

add_library(gianduja_beacon_stop_loopfunc SHARED GiandujaBeaconStopLoopFunc.h GiandujaBeaconStopLoopFunc.cpp 
../../../argos3-Automode/beacon/epuck_beacon.h  ../../../argos3-Automode/beacon/epuck_beacon.cpp
)
target_link_libraries(gianduja_beacon_stop_loopfunc argos3_demiurge_loop_functions)

add_library(gianduja_beacon_decision_loopfunc SHARED GiandujaBeaconDecisionLoopFunc.h GiandujaBeaconDecisionLoopFunc.cpp
../../../argos3-Automode/beacon/epuck_beacon.h  ../../../argos3-Automode/beacon/epuck_beacon.cpp
)
target_link_libraries(gianduja_beacon_decision_loopfunc argos3_demiurge_loop_functions )
