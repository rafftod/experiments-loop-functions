/**
  * @file <loop-functions/example/ExampleAggregationLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "ShelterFiftyFiftyLoopFunc.h"

/****************************************/
/****************************************/

ShelterFiftyFiftyLoopFunction::ShelterFiftyFiftyLoopFunction() {
  m_fWidthShelterArea = 0.2;
  m_fLengthShelterArea = 0.5;
  m_fDistanceBetweenShelters = 1.2;
  m_fObjectiveFunction = 0;

  // Black floor
  m_cBlackFloorX = CRange<Real>(-m_fLengthShelterArea/2, m_fLengthShelterArea/2);     // Width of black area
  m_cBlackFloorY = CRange<Real>(-m_fDistanceBetweenShelters/2, m_fDistanceBetweenShelters/2);   // Length of white area

  // Shelter A
  m_cShelterAY = CRange<Real>(m_cBlackFloorY.GetMin()-m_fWidthShelterArea, m_cBlackFloorY.GetMin());
  m_cShelterAX = CRange<Real>(m_cBlackFloorX.GetMin(), m_cBlackFloorX.GetMax());

  // Shelter B
  m_cShelterBY = CRange<Real>(m_cBlackFloorY.GetMax(), m_cBlackFloorY.GetMax()+m_fWidthShelterArea);
  m_cShelterBX = CRange<Real>(m_cBlackFloorX.GetMin(), m_cBlackFloorX.GetMax());
}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);

  CQuaternion angleWall;
  Real hightWalls = 0.15;

  // Back walls of shelters
  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenterA = new CBoxEntity("wall_back_A",
      CVector3(0, m_fDistanceBetweenShelters/2 + m_fWidthShelterArea, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAX.GetSpan()+0.05, hightWalls));
  AddEntity(*m_pcBoxCenterA);

  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenterB = new CBoxEntity("wall_back_B",
      CVector3(0, -m_fDistanceBetweenShelters/2 - m_fWidthShelterArea, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAX.GetSpan()+0.05, hightWalls));
  AddEntity(*m_pcBoxCenterB);

  // Left wall of shelters
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxLeftA = new CBoxEntity("wall_left_A",
      CVector3(m_cShelterAY.GetSpan()+0.08, m_fDistanceBetweenShelters/2 + (m_fWidthShelterArea/2)+0.01, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAY.GetSpan()+0.03, hightWalls));
  AddEntity(*m_pcBoxLeftA);

  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxLeftB = new CBoxEntity("wall_left_B",
      CVector3(m_cShelterAY.GetSpan()+0.08, -m_fDistanceBetweenShelters/2 - (m_fWidthShelterArea/2)-0.01, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAY.GetSpan()+0.03, hightWalls));
  AddEntity(*m_pcBoxLeftB);

  // Right wall of shelters
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxRightA = new CBoxEntity("wall_right_A",
      CVector3(-m_cShelterAY.GetSpan()-0.08, m_fDistanceBetweenShelters/2 + (m_fWidthShelterArea/2)+0.01, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAY.GetSpan()+0.03, hightWalls));
  AddEntity(*m_pcBoxRightA);

  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxRightB = new CBoxEntity("wall_right_B",
      CVector3(-m_cShelterAY.GetSpan()-0.08, -m_fDistanceBetweenShelters/2 - (m_fWidthShelterArea/2)-0.01, 0.0),
      angleWall,
      false,
      CVector3(0.05, m_cShelterAY.GetSpan()+0.03, hightWalls));
  AddEntity(*m_pcBoxRightB);
}

/****************************************/
/****************************************/

ShelterFiftyFiftyLoopFunction::ShelterFiftyFiftyLoopFunction(const ShelterFiftyFiftyLoopFunction& orig) {}

/****************************************/
/****************************************/

ShelterFiftyFiftyLoopFunction::~ShelterFiftyFiftyLoopFunction() {}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}


/****************************************/
/****************************************/

argos::CColor ShelterFiftyFiftyLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  // Black area
  if (m_cBlackFloorX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and m_cBlackFloorY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
    return CColor::BLACK;
  }

  // Shelter A
  if (m_cShelterAX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and m_cShelterAY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
   return CColor::WHITE;
  }

  // Shelter B
  if (m_cShelterBX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and m_cShelterBY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
   return CColor::WHITE;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::PostExperiment() {
  // CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  // CVector2 cEpuckPosition(0,0);
  // for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
  //   CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
  //   cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
  //                      pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
  // }

  //LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  UInt32 unCurrentScore = 0;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if ((m_cShelterAX.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetX())
            and m_cShelterAY.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetY()))
        or (m_cShelterBX.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetX())
            and m_cShelterBY.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetY()))) {
      unCurrentScore += 1;
    }
  }

  m_fObjectiveFunction += unCurrentScore;
  // LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real ShelterFiftyFiftyLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ShelterFiftyFiftyLoopFunction::GetRandomPosition() {
  Real a;
  Real b;
  Real temp;

  a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
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

REGISTER_LOOP_FUNCTIONS(ShelterFiftyFiftyLoopFunction, "shelter_fifty_fifty_loop_functions");
