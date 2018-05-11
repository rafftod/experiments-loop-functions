#ifndef GIANDUJA_DECISION_LOOP_FUNC
#define GIANDUJA_DECISION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class GiandujaDecisionLoopFunction: public CoreLoopFunctions {
  public:
    GiandujaDecisionLoopFunction();
    GiandujaDecisionLoopFunction(const GiandujaDecisionLoopFunction& orig);
    virtual ~GiandujaDecisionLoopFunction();

    virtual void Destroy();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();

    virtual CVector3 GetRandomPosition();

    Real GetObjectiveFunction();
  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_CCoordRect1;
    CVector2 m_CCoordRect2;
    CVector2 m_CCoordRect2Pos;
    CVector2 m_CCoordRect1Pos;

    UInt8 m_unState;
    UInt32 m_unCostI;
    UInt32 m_unCostO;
    UInt32 m_unTbar;
    UInt32 m_unDecision;
    UInt32 m_unPatchParam;
    Real m_fObjectiveFunction;
};

#endif
