#ifndef GIANDUJA_BEACON_DECISION_LOOP_FUNC
#define GIANDUJA_BEACON_DECISION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "epuck_beacon.h"

using namespace argos;

class GiandujaBeaconDecisionLoopFunction: public CoreLoopFunctions {
  public:
    GiandujaBeaconDecisionLoopFunction();
    GiandujaBeaconDecisionLoopFunction(const GiandujaBeaconDecisionLoopFunction& orig);
    virtual ~GiandujaBeaconDecisionLoopFunction();

    virtual void Destroy();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual CVector3 GetRandomPosition();

    Real GetObjectiveFunction();
    void PlaceBeacon();
    void ExtractTime();
    void ChangeMessage();
    void ExtractMessage();

  private:
    Real m_fRadius;
    CVector2 m_CCoordRect2Pos;
    CVector2 m_CCoordRect1Pos;
    CVector2 m_CCoordLimit;

    UInt8 m_unState;
    UInt32 m_unCostI;
    UInt32 m_unTbar;
    UInt32 m_unTbar2;
    Real m_fObjectiveFunction;
    UInt32 m_unTime;
    UInt32 m_unMessage;
};

#endif
