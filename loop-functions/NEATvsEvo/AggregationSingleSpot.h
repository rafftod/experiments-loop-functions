/**
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef AGGREGATION_SINGLE_SPOT
#define AGGREGATION_SINGLE_SPOT

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class AggregationSingleSpot: public CoreLoopFunctions {
  public:
    AggregationSingleSpot();
    AggregationSingleSpot(const AggregationSingleSpot& orig);
    virtual ~AggregationSingleSpot();

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
    UInt32 m_unScoreSpot1;
    Real m_fObjectiveFunction;
    Real m_fHeightNest;
};

#endif
