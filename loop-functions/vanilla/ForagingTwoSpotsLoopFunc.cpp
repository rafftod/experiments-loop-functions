/**
  * @file <loop-functions/ForagingTwoSpotsLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @license MIT License
  */

#include "ForagingTwoSpotsLoopFunc.h"

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::ForagingTwoSpotsLoopFunction() {
  m_fRadius = 0.15;
  m_fNestLimit = -0.6;
  m_cCoordSpot1 = CVector2(0.75,0);
  m_cCoordSpot2 = CVector2(-0.75,0);
  m_fObjectiveFunction = 0;
  m_bInitializationStep = true;
  m_punFoodData  = NULL;
  m_cRegex = std::regex("epuck_?([0-9]+)_?([0-9]+)?");
}

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::ForagingTwoSpotsLoopFunction(const ForagingTwoSpotsLoopFunction& orig) {
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/

ForagingTwoSpotsLoopFunction::~ForagingTwoSpotsLoopFunction() {
  delete[] m_punFoodData;
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Destroy() {}

/****************************************/
/****************************************/

argos::CColor ForagingTwoSpotsLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  // if (vCurrentPoint.GetY() > m_fNestLimit)
  //   return CColor::GRAY50;
  // else
  //   return CColor::WHITE;
  if (vCurrentPoint.GetY() < m_fNestLimit) {
    Real fMaxDim = 1.231;
    Real fLengthWall = 0.66;
    Real fWidthWall = 0.01;
    Real fPI = 3.14159265;
    CVector2 cCenter(0, m_fNestLimit);
    CVector2 cTop(1.066 + fWidthWall, m_fNestLimit);
    CVector2 cBottom(-1.066 - fWidthWall, m_fNestLimit);
    CVector2 cAngle1(1.066 - ARGOS_COS(60*fPI/180.0f)*fLengthWall/2 + fWidthWall, m_fNestLimit - (ARGOS_SIN(60*fPI/180.0f)*fLengthWall/2) - fWidthWall);
    CVector2 cAngle2(fLengthWall/2 + fWidthWall, -fMaxDim);
    CVector2 cAngle3(-(fLengthWall/2 + fWidthWall), -fMaxDim);
    CVector2 cAngle4(-1.066 + ARGOS_COS(60*fPI/180.0f)*fLengthWall/2 - fWidthWall, m_fNestLimit - (ARGOS_SIN(60*fPI/180.0f)*fLengthWall/2) - fWidthWall);
    if ((IsWithinTriangle(vCurrentPoint, cCenter, cTop, cAngle1)) ||
        (IsWithinTriangle(vCurrentPoint, cCenter, cAngle2, cAngle3)) ||
        (IsWithinTriangle(vCurrentPoint, cCenter, cAngle1, cAngle2)) ||
        (IsWithinTriangle(vCurrentPoint, cCenter, cAngle3, cAngle4)) ||
        (IsWithinTriangle(vCurrentPoint, cCenter, cAngle4, cBottom))) {
      return CColor::WHITE;
    } else {
      return CColor::GRAY50;
    }
  } else {
    return CColor::GRAY50;
  }
}

/****************************************/
/****************************************/

Real ForagingTwoSpotsLoopFunction::AreaTriangle(CVector2& c_point_a, CVector2& c_point_b, CVector2& c_point_c) {
  Real fArea = Abs(c_point_a.GetX()*(c_point_b.GetY()-c_point_c.GetY()) + c_point_b.GetX()*(c_point_c.GetY()-c_point_a.GetY()) + c_point_c.GetX()*(c_point_a.GetY()-c_point_b.GetY()))/2;
  return fArea;
}

/****************************************/
/****************************************/

bool ForagingTwoSpotsLoopFunction::IsWithinTriangle(CVector2& c_point_q, CVector2& c_point_a, CVector2& c_point_b, CVector2& c_point_c) {
  Real fAreaTriangle = AreaTriangle(c_point_a, c_point_b, c_point_c);
  Real fAreaABQ = AreaTriangle(c_point_a, c_point_b, c_point_q);
  Real fAreaBCQ = AreaTriangle(c_point_b, c_point_c, c_point_q);
  Real fAreaACQ = AreaTriangle(c_point_a, c_point_c, c_point_q);

  if (Abs(fAreaTriangle - (fAreaABQ + fAreaACQ + fAreaBCQ)) < 0.0001) {
    return true;
  } else {
    return false;
  }
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::Reset() {
  CoreLoopFunctions::Reset();
  std::ios::sync_with_stdio(false);
  if (m_punFoodData == NULL) {
    m_punFoodData = new UInt32[m_unNumberRobots+1];
  }
  for (UInt32 i = 0; i <= m_unNumberRobots; i++) {
    m_punFoodData[i] = 0;
  }

  m_fObjectiveFunction = 0;
  m_bInitializationStep = false;
}

/****************************************/
/****************************************/

void ForagingTwoSpotsLoopFunction::PostStep() {
  //m_unNumberRobots = 70;

  UInt32 score_temp = m_fObjectiveFunction;

  if (m_bInitializationStep) {
    m_punFoodData = new UInt32[m_unNumberRobots+1];
    for (UInt32 i = 0; i <= m_unNumberRobots; i++) {
      m_punFoodData[i] = 0;
    }
    m_bInitializationStep = false;
  }
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  CVector2 cEpuckPosition(0,0);
  UInt32 unId=0;
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);

    // expects the id to be in the following form: epuck{robotId} or epuck_{robotId}_{tagId} or epuck_{robotId}
    std::string strRobotId = pcEpuck->GetId();
    std::smatch cMatch;
    bool bMatchFound = std::regex_match(strRobotId, cMatch , m_cRegex);
    if (bMatchFound) {
      unId = std::stoi(cMatch[1].str());
    }

    //unId = atoi(strRobotId.c_str());
    cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                       pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
    Real fDistanceSpot2 = (m_cCoordSpot2 - cEpuckPosition).Length();
    if (fDistanceSpot1 <= m_fRadius) {
      m_punFoodData[unId] = 1;
    } else if (fDistanceSpot2 <= m_fRadius){
      m_punFoodData[unId] = 1;
    } else if (cEpuckPosition.GetY() <= m_fNestLimit) {
      m_fObjectiveFunction = m_fObjectiveFunction + m_punFoodData[unId];
      m_punFoodData[unId] = 0;
      // LOG << "Obj " << m_fObjectiveFunction << std::endl;
    }
  }

  if (score_temp != m_fObjectiveFunction) {
     //LOGERR << "Obj " << m_fObjectiveFunction << std::endl;
  }
}

/****************************************/
/****************************************/

Real ForagingTwoSpotsLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ForagingTwoSpotsLoopFunction::GetRandomPosition() {
  Real temp;
  Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
  Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
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

REGISTER_LOOP_FUNCTIONS(ForagingTwoSpotsLoopFunction, "foraging_loop_functions");
