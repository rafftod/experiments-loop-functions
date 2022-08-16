/**
 * @file <loop-functions/example/AggregationLoopFunc.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AGGREGATION_ONE_SPOT_LOOP_FUNC
#define AGGREGATION_ONE_SPOT_LOOP_FUNC

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include "../../../src/CoreLoopFunctions.h"

using namespace argos;

class AggregationOneSpotLoopFunc : public CoreLoopFunctions
{
public:
  AggregationOneSpotLoopFunc();
  AggregationOneSpotLoopFunc(const AggregationOneSpotLoopFunc &orig);
  virtual ~AggregationOneSpotLoopFunc();

  virtual void Destroy();
  virtual void Init(TConfigurationNode &t_tree);

  virtual argos::CColor GetFloorColor(const argos::CVector2 &c_position_on_plane);
  virtual void PostExperiment();
  virtual void PostStep();
  virtual void Reset();

  Real GetObjectiveFunction();

  CVector3 GetRandomPosition();

  void PositionRobots();

  void ArrestTrespassers();
  CVector3 GetJailPosition();

private:
  Real m_fRadius;
  CVector2 m_cCoordSpot1;

  UInt32 m_unScoreSpot1;
  Real m_fObjectiveFunction;
};

#endif
