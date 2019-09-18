/**
  * @file <loop-functions/AggregationTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "AggregationSingleSpot.h"

/****************************************/
/****************************************/

AggregationSingleSpot::AggregationSingleSpot() {
  m_fRadius = 0.3;
  m_cCoordSpot1 = CVector2(0,-0.7);
  m_unScoreSpot1 = 0;
  m_fObjectiveFunction = 0;
  m_fHeightNest = 0.6;
}

/****************************************/
/****************************************/

AggregationSingleSpot::AggregationSingleSpot(const AggregationSingleSpot& orig) {}

/****************************************/
/****************************************/

void AggregationSingleSpot::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
    } catch(std::exception e) {
    }
}

/****************************************/
/****************************************/


AggregationSingleSpot::~AggregationSingleSpot() {}

/****************************************/
/****************************************/

void AggregationSingleSpot::Destroy() {}

/****************************************/
/****************************************/

argos::CColor AggregationSingleSpot::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void AggregationSingleSpot::Reset() {
  m_fObjectiveFunction = 0;
  m_unScoreSpot1 = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AggregationSingleSpot::PostStep() {
}

/****************************************/
/****************************************/

void AggregationSingleSpot::PostExperiment() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      m_unScoreSpot1 += 1;
    }
  }

  m_fObjectiveFunction = m_unScoreSpot1/(Real) m_unNumberRobots;
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AggregationSingleSpot::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationSingleSpot::GetRandomPosition() {
  Real temp, a, b, fPosX, fPosY;
  bool bPlaced = false;
  do {
      a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
      // If b < a, swap them
      if (b < a) {
        temp = a;
        a = b;
        b = temp;
      }
      fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a/b));
      fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a/b));

      if (fPosY >= m_fHeightNest) {
        bPlaced = true;
      }
  } while(!bPlaced);
  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(AggregationSingleSpot, "aggregation_single");
