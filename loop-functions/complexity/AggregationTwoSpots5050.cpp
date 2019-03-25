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
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttribute(cParametersNode, "output_file", m_strOutputFile);
      OpenFile();
    } catch(std::exception e) {
    }
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

void AggregationTwoSpots5050::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  UInt32 unNumberRobotsSpotA = 0;
  UInt32 unNumberRobotsSpotB = 0;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    NeuralNetworkRM1Dot1Binary& cController = dynamic_cast<NeuralNetworkRM1Dot1Binary&>(pcEpuck->GetControllableEntity().GetController());
    m_cTrace << cController.GetState();
    LOG << cController.GetState();

    /* Fill Fitness file */
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      unNumberRobotsSpotA += 1;
    } else if (fDistanceSpot2 <= m_fRadius){
      unNumberRobotsSpotB += 1;
    }
  }
  m_cTrace << std::endl;
  m_cFitness << unNumberRobotsSpotA << "\t" << unNumberRobotsSpotB << "\t" << Min(unNumberRobotsSpotA, unNumberRobotsSpotB)/((Real) m_unNumberRobots/2) << std::endl;
  LOG << std::endl;
}

/****************************************/
/****************************************/

void AggregationTwoSpots5050::OpenFile() {
  std::stringstream stringStream;
  stringStream << m_strOutputFile + ".trace";
  std::string strTrace = stringStream.str();
  LOGERR << strTrace << std::endl;
  m_cTrace.open(strTrace.c_str(), std::ios_base::trunc | std::ios_base::out);

  std::stringstream stringStream2;
  stringStream2 << m_strOutputFile + ".fitness";
  std::string strFitness = stringStream2.str();
  LOGERR << strFitness << std::endl;
  m_cFitness.open(strFitness.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_cFitness << "SpotA\tSpotB\tScore" << std::endl;
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
  m_cTrace.close();
  m_cFitness.close();
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
