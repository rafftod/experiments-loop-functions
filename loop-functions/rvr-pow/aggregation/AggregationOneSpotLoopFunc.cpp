/**
 * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @license MIT License
 */

#include "AggregationOneSpotLoopFunc.h"

/****************************************/
/****************************************/

AggregationOneSpotLoopFunc::AggregationOneSpotLoopFunc()
{
  m_fRadius = 0.5;
  m_cCoordSpot1 = CVector2(0, 0);
  m_unScoreSpot1 = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

AggregationOneSpotLoopFunc::AggregationOneSpotLoopFunc(const AggregationOneSpotLoopFunc &orig) {}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::Init(TConfigurationNode &t_tree)
{
  CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/

AggregationOneSpotLoopFunc::~AggregationOneSpotLoopFunc() {}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationOneSpotLoopFunc::GetFloorColor(const argos::CVector2 &c_position_on_plane)
{
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius)
  {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::Reset()
{
  m_fObjectiveFunction = 0;
  m_unScoreSpot1 = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::PostStep()
{
  ArrestTrespassers();
  CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
  CVector2 cEpuckPosition(0, 0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
  {
    CRVREntity *pcEpuck = any_cast<CRVREntity *>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    if (fDistanceSpot1 > m_fRadius)
    {
      // m_unScoreSpot1 += 1;
    }
  }
}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::PostExperiment()
{
  m_unScoreSpot1 = 0;
  CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
  CVector2 cEpuckPosition(0, 0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
  {
    CRVREntity *pcEpuck = any_cast<CRVREntity *>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius)
    {
      m_unScoreSpot1 += 1;
    }
  }
  // LOG << "Final value : "<< m_fObjectiveFunction << std::endl;
  LOG << "Score " << m_unScoreSpot1 << std::endl;
}

/****************************************/
/****************************************/

Real AggregationOneSpotLoopFunc::GetObjectiveFunction()
{
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationOneSpotLoopFunc::GetRandomPosition()
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

void AggregationOneSpotLoopFunc::PositionRobots()
{
  Real a;
  Real b;
  Real temp;

  CRVREntity *pcEpuck;
  UInt32 unTrials;
  bool bPlaced = false;

  for (UInt32 i = 1; i < m_unNumberRobots + 1; ++i)
  {
    std::ostringstream id;
    id << "rvr" << i;
    pcEpuck = new CRVREntity(id.str().c_str(),
                             "automode",
                             CVector3(0, 0, 0),
                             CQuaternion().FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO));
    AddEntity(*pcEpuck);
    // Choose position at random
    unTrials = 0;
    do
    {
      ++unTrials;
      a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      // If b < a, swap them
      if (b < a)
      {
        temp = a;
        a = b;
        b = temp;
      }
      Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a / b));
      Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a / b));
      bPlaced = MoveEntity((*pcEpuck).GetEmbodiedEntity(),
                           CVector3(fPosX, fPosY, 0),
                           CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)),
                                                         CRadians::ZERO, CRadians::ZERO),
                           false);
    } while (!bPlaced && unTrials < 100);
    if (!bPlaced)
    {
      THROW_ARGOSEXCEPTION("Can't place robot #" << i);
    }
  }
}

/****************************************/
/****************************************/

void AggregationOneSpotLoopFunc::ArrestTrespassers()
{
  CRVREntity *pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
  {
    pcEpuck = any_cast<CRVREntity *>(it->second);
    // Choose position at random
    Real posY = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
    Real posX = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    if (pow(posY, 2.0) + pow(posX, 2.0) > pow(1.250, 2.0) && posY < 1.900)
    {
      unTrials = 0;
      do
      {
        ++unTrials;
        CVector3 cEpuckPosition = GetJailPosition();
        bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                             cEpuckPosition,
                             CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)),
                                                           CRadians::ZERO, CRadians::ZERO),
                             false);
      } while (!bPlaced && unTrials < 100);
      if (!bPlaced)
      {
        THROW_ARGOSEXCEPTION("Can't place robot");
      }
    }
  }
}

CVector3 AggregationOneSpotLoopFunc::GetJailPosition()
{
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

  Real fPosX = a * 2.5 - 1.250;
  Real fPosY = b * 0.5 + 1.950;

  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(AggregationOneSpotLoopFunc, "aggregation_one_spot_loop_functions");
