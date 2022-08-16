#ifndef GIANDUJA_AGGREGATION_LOOP_FUNC
#define GIANDUJA_AGGREGATION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

using namespace argos;

class GiandujaAggregationLoopFunction: public CoreLoopFunctions {
  public:
    GiandujaAggregationLoopFunction();
    GiandujaAggregationLoopFunction(const GiandujaAggregationLoopFunction& orig);
    virtual ~GiandujaAggregationLoopFunction();

    virtual void Destroy();
    virtual void Reset();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();

    Real GetObjectiveFunction();
    void PlaceLight();

    virtual CVector3 GetRandomPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_cCoordSpot2;

    UInt32 m_unCostSpot1;
    Real m_fObjectiveFunction;
};

#endif
