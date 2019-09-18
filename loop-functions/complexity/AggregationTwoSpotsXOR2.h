/**
  * @file <loop-functions/example/AggregationLoopFunc.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AGGREGATION_TWO_SPOTS_XOR2
#define AGGREGATION_TWO_SPOTS_XOR2

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class AggregationTwoSpotsXOR2: public CoreLoopFunctions {
  public:
    AggregationTwoSpotsXOR2();
    AggregationTwoSpotsXOR2(const AggregationTwoSpotsXOR2& orig);
    virtual ~AggregationTwoSpotsXOR2();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();

  private:
    Real m_fRadius;
    CVector2 m_cCoordSpot1;
    CVector2 m_cCoordSpot2;

    UInt32 m_unScoreSpot1;
    UInt32 m_unScoreSpot2;
    Real m_fObjectiveFunction;
};

#endif
