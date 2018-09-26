/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "AggregationTwoSpots5050.h"

/****************************************/
/****************************************/

AggregationTwoSpots5050::AggregationTwoSpots5050() {
  m_fRadius = 0.3;
  m_cCoordSpot1 = CVector2(0.65,0);
  m_cCoordSpot2 = CVector2(-0.65,0);
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

AggregationTwoSpots5050::AggregationTwoSpots5050(const AggregationTwoSpots5050& orig) {}

/****************************************/
/****************************************/

void AggregationTwoSpots5050::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/


AggregationTwoSpots5050::~AggregationTwoSpots5050() {}

/****************************************/
/****************************************/

void AggregationTwoSpots5050::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationTwoSpots5050::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void AggregationTwoSpots5050::Reset() {
  m_fObjectiveFunction = 0;
  m_unScoreSpot1 = 0;
  m_unScoreSpot2 = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AggregationTwoSpots5050::PostExperiment() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      m_unScoreSpot1 += 1;
    } else if (fDistanceSpot2 <= m_fRadius){
      m_unScoreSpot2 += 1;
    }
  }

  m_fObjectiveFunction = Min(m_unScoreSpot1, m_unScoreSpot2)/((Real) m_unNumberRobots/2);
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AggregationTwoSpots5050::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationTwoSpots5050::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  // If b < a, swap them
  if (b < a) {
    temp = a;
    a = b;
    b = temp;
  }
  Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
  Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(AggregationTwoSpots5050, "aggregation_5050");
