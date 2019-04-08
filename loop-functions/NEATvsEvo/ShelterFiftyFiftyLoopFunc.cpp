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
  m_fWidthShelterArea = 0.14;
  m_fLengthShelterArea = 0.35;
  m_fDistanceBetweenShelters = 1.2;
  m_fObjectiveFunction = 0;

  // Black floor
  m_cBlackFloorX = CRange<Real>(-m_fLengthShelterArea/2, m_fLengthShelterArea/2);     // Width of black area
  m_cBlackFloorY = CRange<Real>(-m_fDistanceBetweenShelters/2, m_fDistanceBetweenShelters/2);   // Length of white area

  // Shelter Left (close to light)
  m_cShelterLeftY = CRange<Real>(m_cBlackFloorY.GetMin()-m_fWidthShelterArea, m_cBlackFloorY.GetMin());
  m_cShelterLeftX = CRange<Real>(m_cBlackFloorX.GetMin(), m_cBlackFloorX.GetMax());

  // Shelter Right
  m_cShelterRightY = CRange<Real>(m_cBlackFloorY.GetMax(), m_cBlackFloorY.GetMax()+m_fWidthShelterArea);
  m_cShelterRightX = CRange<Real>(m_cBlackFloorX.GetMin(), m_cBlackFloorX.GetMax());
}

/****************************************/
/****************************************/

void ShelterFiftyFiftyLoopFunction::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);

  CQuaternion angleWall;
  Real fHeightWalls = 0.15;
  Real fWidthWalls = 0.05;

  // Back walls of shelters
  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenterLeft = new CBoxEntity("wall_back_left",
      CVector3(0, m_fDistanceBetweenShelters/2 + m_fWidthShelterArea + (fWidthWalls/2), 0.0), // Position
      angleWall,
      false,
      CVector3(fWidthWalls, m_fLengthShelterArea + (fWidthWalls*2), fHeightWalls));   // Size
  AddEntity(*m_pcBoxCenterLeft);

  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenterRight = new CBoxEntity("wall_back_right",
      CVector3(0, -m_fDistanceBetweenShelters/2 - m_fWidthShelterArea - (fWidthWalls/2), 0.0),
      angleWall,
      false,
      CVector3(fWidthWalls, m_fLengthShelterArea + (fWidthWalls*2), fHeightWalls));
  AddEntity(*m_pcBoxCenterRight);

  // Top walls of shelters
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxTopLeft = new CBoxEntity("wall_top_left",
      CVector3(m_fLengthShelterArea/2 + (fWidthWalls/2), m_fDistanceBetweenShelters/2 + (m_fWidthShelterArea/2), 0.0),
      angleWall,
      false,
      CVector3(fWidthWalls, m_fWidthShelterArea, fHeightWalls));
  AddEntity(*m_pcBoxTopLeft);

  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxTopRight = new CBoxEntity("wall_top_right",
      CVector3(m_fLengthShelterArea/2 + (fWidthWalls/2), -m_fDistanceBetweenShelters/2 - (m_fWidthShelterArea/2), 0.0),
      angleWall,
      false,
      CVector3(fWidthWalls, m_fWidthShelterArea, fHeightWalls));
  AddEntity(*m_pcBoxTopRight);

  // Bottom walls of shelters
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxBottomLeft = new CBoxEntity("wall_bottom_left",
      CVector3(-m_fLengthShelterArea/2 - (fWidthWalls/2), m_fDistanceBetweenShelters/2 + (m_fWidthShelterArea/2), 0.0),
      angleWall,
      false,
      CVector3(fWidthWalls, m_fWidthShelterArea, fHeightWalls));
  AddEntity(*m_pcBoxBottomLeft);

  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxBottomRight = new CBoxEntity("wall_bottom_right",
      CVector3(-m_fLengthShelterArea/2 - (fWidthWalls/2), -m_fDistanceBetweenShelters/2 - (m_fWidthShelterArea/2), 0.0),
      angleWall,
      false,
      CVector3(fWidthWalls, m_fWidthShelterArea, fHeightWalls));
  AddEntity(*m_pcBoxBottomRight);
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
  if (m_cBlackFloorX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and
      m_cBlackFloorY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
    return CColor::BLACK;
  }

  // Shelter Left
  if (m_cShelterLeftX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and
      m_cShelterLeftY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
   return CColor::WHITE;
  }

  // Shelter Right
  if (m_cShelterRightX.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetX()) and
      m_cShelterRightY.WithinMinBoundIncludedMaxBoundIncluded(vCurrentPoint.GetY())) {
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
  UInt32 unNbRobotLeft = 0;
  UInt32 unNbRobotRight = 0;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if (m_cShelterLeftX.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetX())
            and m_cShelterLeftY.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetY())) {
      unNbRobotLeft += 1;
    }
    if (m_cShelterRightX.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetX())
            and m_cShelterRightY.WithinMinBoundIncludedMaxBoundIncluded(cEpuckPosition.GetY())) {
      unNbRobotRight += 1;
    }
  }

  //LOG << "L: " << unNbRobotLeft << "  R: " << unNbRobotRight << std::endl;
  m_fObjectiveFunction += (unNbRobotLeft + unNbRobotRight);
  //LOG << m_fObjectiveFunction << std::endl;
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
