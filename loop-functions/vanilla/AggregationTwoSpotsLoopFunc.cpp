/**
 * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @license MIT License
 */

#include "AggregationTwoSpotsLoopFunc.h"

/****************************************/
/****************************************/

AggregationTwoSpotsLoopFunction::AggregationTwoSpotsLoopFunction()
{
  m_fRadius = 0.3;
  m_cCoordSpot1 = CVector2(0.55, 0);
  m_cCoordSpot2 = CVector2(-0.55, 0);
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

AggregationTwoSpotsLoopFunction::AggregationTwoSpotsLoopFunction(const AggregationTwoSpotsLoopFunction &orig) {}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Init(TConfigurationNode &t_tree)
{
  CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/

AggregationTwoSpotsLoopFunction::~AggregationTwoSpotsLoopFunction() {}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationTwoSpotsLoopFunction::GetFloorColor(const argos::CVector2 &c_position_on_plane)
{
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius)
  {
    return CColor::BLACK;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius)
  {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::Reset()
{
  m_fObjectiveFunction = 0;
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AggregationTwoSpotsLoopFunction::PostExperiment()
{
  CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0, 0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
  {
    CEPuckEntity *pcEpuck = any_cast<CEPuckEntity *>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius)
    {
      m_unScoreSpot1 += 1;
    }
    else if (fDistanceSpot2 <= m_fRadius)
    {
      m_unScoreSpot2 += 1;
    }
  }

  m_fObjectiveFunction = Max(m_unScoreSpot1, m_unScoreSpot2) / (Real)m_unNumberRobots;
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AggregationTwoSpotsLoopFunction::GetObjectiveFunction()
{
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationTwoSpotsLoopFunction::GetRandomPosition()
{
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  // If b < a, swap them
  if (b < a)
  {
    temp = a;
    a = b;
    b = temp;
  }
  Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a / b));
  Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a / b));

  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(AggregationTwoSpotsLoopFunction, "aggregation_loop_functions");
