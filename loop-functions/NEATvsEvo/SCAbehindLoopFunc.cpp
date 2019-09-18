/**
  * @file <loop-functions/example/ShelterConstrainedAccessLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "SCAbehindLoopFunc.h"

/****************************************/
/****************************************/

SCAbehindLoopFunction::SCAbehindLoopFunction() {
  m_fSpotRadius = 0.3;
  m_cCoordBlackSpot = CVector2(0.8, 0.0);
  m_cCoordWhiteSpot = CVector2(-0.8, 0.0);
  m_fObjectiveFunction = 0;
  m_fWidthShelter = 0.5;
  m_fHeightShelter = 0.3;
  m_fHeightWallsShelter = 0.07;
  m_fWidthWallsShelter = 0.045;
  m_cPositionShelter = CVector2(0,0);
}

/****************************************/
/****************************************/

SCAbehindLoopFunction::SCAbehindLoopFunction(const SCAbehindLoopFunction& orig) {}

/****************************************/
/****************************************/

SCAbehindLoopFunction::~SCAbehindLoopFunction() {}

/****************************************/

/****************************************/
void SCAbehindLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void SCAbehindLoopFunction::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void SCAbehindLoopFunction::Init(TConfigurationNode& t_tree) {
  CoreLoopFunctions::Init(t_tree);

  CQuaternion angleWall;

  // Center
  angleWall.FromEulerAngles(CRadians::PI_OVER_TWO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxCenter = new CBoxEntity("wall_center",
      CVector3(0, m_cPositionShelter.GetY() + (m_fHeightShelter/2) + (m_fWidthWallsShelter/2), 0.0),
      angleWall,
      false,
      CVector3(m_fWidthWallsShelter, m_fWidthShelter+m_fWidthWallsShelter, m_fHeightWallsShelter));
  AddEntity(*m_pcBoxCenter);


  // Left
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxLeft = new CBoxEntity("wall_left",
      CVector3(-m_fWidthShelter/2-0.05/4, m_cPositionShelter.GetY()+m_fWidthWallsShelter/2, 0.0),
      angleWall,
      false,
      CVector3(m_fWidthWallsShelter, m_fHeightShelter+m_fWidthWallsShelter, m_fHeightWallsShelter));
  AddEntity(*m_pcBoxLeft);

  // Right
  m_pcBoxRight = new CBoxEntity("wall_right",
      CVector3(m_fWidthShelter/2+0.05/4, m_cPositionShelter.GetY()+m_fWidthWallsShelter/2, 0.0),
      angleWall,
      false,
      CVector3(m_fWidthWallsShelter, m_fHeightShelter+m_fWidthWallsShelter, m_fHeightWallsShelter));
  AddEntity(*m_pcBoxRight);
}

/****************************************/
/****************************************/

argos::CColor SCAbehindLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
  if (d <= m_fSpotRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordWhiteSpot - vCurrentPoint).Length();
  if (d <= m_fSpotRadius) {
    return CColor::BLACK;
  }

  if (IsInShelter(vCurrentPoint)) {
    return CColor::WHITE;
  }

  return CColor::GRAY50;
}

/****************************************/
/****************************************/

bool SCAbehindLoopFunction::IsInShelter(CVector2& c_position) {
  Real fMaximalXCoord = m_fWidthShelter / 2;
  Real fMaximalYCoord = (m_fHeightShelter / 2) + m_cPositionShelter.GetY();
  if (c_position.GetX() > -fMaximalXCoord && c_position.GetX() < fMaximalXCoord) {
    if (c_position.GetY() > -fMaximalYCoord && c_position.GetY() < fMaximalYCoord) {
      return true;
    }
  }
  return false;
}

/****************************************/
/****************************************/

void SCAbehindLoopFunction::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    if (IsInShelter(cEpuckPosition)) {
      m_fObjectiveFunction += 1;
    }
  }
}

/****************************************/
/****************************************/

void SCAbehindLoopFunction::PostExperiment() {
  LOG << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real SCAbehindLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 SCAbehindLoopFunction::GetRandomPosition() {
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

    if (fPosY >= m_fHeightShelter/2) {
      bPlaced = true;
    }
  } while (!bPlaced);
  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(SCAbehindLoopFunction, "sca_behind_loop_functions");
