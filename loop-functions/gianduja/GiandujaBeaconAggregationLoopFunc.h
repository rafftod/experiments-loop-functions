#ifndef GIANDUJA_BEACON_AGGREGATION_LOOP_FUNC
#define GIANDUJA_BEACON_AGGREGATION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "../../../robots-thesis/src/beacon_epuck/epuck_beacon.h"

using namespace argos;

class GiandujaBeaconAggregationLoopFunction: public CoreLoopFunctions {
  public:
    GiandujaBeaconAggregationLoopFunction();
    GiandujaBeaconAggregationLoopFunction(const GiandujaBeaconAggregationLoopFunction& orig);
    virtual ~GiandujaBeaconAggregationLoopFunction();

    virtual void Destroy();
    virtual void Reset();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();

    Real GetObjectiveFunction();
    void PlaceLight();
    void ExtractTime();
    void PlaceBeacon();

    virtual CVector3 GetRandomPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_cCoordSpot2;

    UInt32 m_unCostSpot1;
    UInt32 m_Tbar;
    UInt8 m_State;
    Real m_fObjectiveFunction;
    UInt32 m_unTime;
};

#endif
