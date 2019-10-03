/**
  * @file <loop-functions/DirectionalGate.cpp>
  *
  * @author Ken H - <khasselm@ulb.ac.be>
  *
  * @license MIT License
  */

#include "DirectionalGate.h"

/****************************************/
/****************************************/

DirectionalGate::DirectionalGate() {
  m_fObjectiveFunction = 0;
  m_fHeightGateZone = 0;
  m_fWidthShelter = 0.5;
  m_fHeightShelter = 0.1;
  m_fHeightWallsShelter = 0.07;
  m_fWidthWallsShelter = 0.045;
  m_cPositionShelter = CVector2(0,0);
}

/****************************************/
/****************************************/

DirectionalGate::DirectionalGate(const DirectionalGate& orig) {}

/****************************************/
/****************************************/

void DirectionalGate::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    TConfigurationNode cParametersNode;
    try {
      cParametersNode = GetNode(t_tree, "params");
      GetNodeAttributeOrDefault(cParametersNode, "gate_width", m_fWidthShelter, (Real) 0.5);
      GetNodeAttributeOrDefault(cParametersNode, "gate_length", m_fHeightShelter, (Real) 0.5);
    } catch(std::exception e) {
    }

  CQuaternion angleWall;

  // Left
  angleWall.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::ZERO);
  m_pcBoxLeft = new CBoxEntity("wall_left",
      CVector3(-m_fWidthShelter/2, m_cPositionShelter.GetY()+m_fHeightShelter/6, 0.0),
      angleWall,
      false,
      CVector3(m_fWidthWallsShelter, m_fHeightShelter, m_fHeightWallsShelter));
  AddEntity(*m_pcBoxLeft);

  // Right
  m_pcBoxRight = new CBoxEntity("wall_right",
      CVector3(m_fWidthShelter/2, m_cPositionShelter.GetY()+m_fHeightShelter/6, 0.0),
      angleWall,
      false,
      CVector3(m_fWidthWallsShelter, m_fHeightShelter, m_fHeightWallsShelter));
  AddEntity(*m_pcBoxRight);

}

/****************************************/
/****************************************/


DirectionalGate::~DirectionalGate() {}

/****************************************/
/****************************************/

void DirectionalGate::Destroy() {}

/****************************************/
/****************************************/

argos::CColor DirectionalGate::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  /* Real d = (m_cCoordSpot1 - vCurrentPoint).Length(); */
  if ( vCurrentPoint.GetX() < m_cPositionShelter.GetX()+m_fWidthShelter/2 && vCurrentPoint.GetX() > m_cPositionShelter.GetX()-m_fWidthShelter/2) {
      if (vCurrentPoint.GetY() > m_cPositionShelter.GetY()+m_fHeightShelter/3) {
        return CColor::BLACK;
      } else if (vCurrentPoint.GetY() > m_cPositionShelter.GetY()-m_fHeightShelter/3) {
        return CColor::WHITE;
      }
  }

  return CColor::GRAY50;
}


/****************************************/
/****************************************/

void DirectionalGate::Reset() {
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void DirectionalGate::PostStep() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

  if (cEpuckPosition.GetX() < m_fWidthShelter/2 && cEpuckPosition.GetX() > -m_fWidthShelter/2) {
      if (m_tOldPosPoints[pcEpuck].GetY() < m_fHeightGateZone && cEpuckPosition.GetY() > m_fHeightGateZone) {
        m_fObjectiveFunction-=1;
        LOG << "Score = " << m_fObjectiveFunction << std::endl;
      }
      else if (m_tOldPosPoints[pcEpuck].GetY() > m_fHeightGateZone && cEpuckPosition.GetY() < m_fHeightGateZone) {
        m_fObjectiveFunction+=1;
        LOG << "Score = " << m_fObjectiveFunction << std::endl;
      }
  }
  m_tOldPosPoints[pcEpuck] = cEpuckPosition;
  }
}

/****************************************/
/****************************************/

void DirectionalGate::PostExperiment() {
  /* m_fObjectiveFunction = m_unScoreSpot1/(Real) m_unNumberRobots; */
  if (m_fObjectiveFunction < 0) {
      m_fObjectiveFunction = 0;
  }
  LOG << "Score = " << m_fObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real DirectionalGate::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 DirectionalGate::GetRandomPosition() {
  Real temp, a, b, fPosX, fPosY;
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
  
  return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(DirectionalGate, "directional_gate");
