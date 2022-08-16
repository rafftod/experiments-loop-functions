#ifndef GIANDUJA_BEACON_AGGREGATION_LOOP_FUNC
#define GIANDUJA_BEACON_AGGREGATION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include "epuck_beacon.h"

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
    void PlaceBeacon();
    void SetMessageBeacon();
    void ExtractMessage();

    virtual CVector3 GetRandomPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_cCoordSpot2;

    SInt32 m_unCostSpot1;
    SInt32 m_unCostSpot2;
    Real m_fObjectiveFunction;
    UInt32 m_unMesParam;
    UInt8 m_unMes;
    UInt32 m_unTime;
};

#endif
