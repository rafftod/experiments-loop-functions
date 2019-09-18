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
  m_cCoordSpot1 = CVector2(0,0);
  m_unScoreSpot1 = 0;
  m_fObjectiveFunction = 0;
  m_strOutputFile = "NoName";
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
      GetNodeAttribute(cParametersNode, "output_file", m_strOutputFile);
      OpenFile();
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
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  UInt32 unNumberRobotsOnPoint = 0;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    /* Fill Trace file */
    NeuralNetworkRM1Dot1Binary& cController = dynamic_cast<NeuralNetworkRM1Dot1Binary&>(pcEpuck->GetControllableEntity().GetController());
    m_cTrace << cController.GetState();
    LOG << cController.GetState();

    /* Fill Fitness file */
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      unNumberRobotsOnPoint += 1;
    }
  }
  m_cTrace << std::endl;
  m_cFitness << unNumberRobotsOnPoint << std::endl;
  LOG << std::endl;
}

/****************************************/
/****************************************/

void AggregationSingleSpot::OpenFile() {
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
  m_cFitness << "Score" << std::endl;
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
  m_cTrace.close();
  m_cFitness.close();
}

/****************************************/
/****************************************/

Real AggregationSingleSpot::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AggregationSingleSpot::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(AggregationSingleSpot, "aggregation_single");
