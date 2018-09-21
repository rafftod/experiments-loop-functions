/**
  * @file <loop-functions/example/GuidedShelter.cpp>
  *
  * @author Jonas Kuckling - <jonas.kuckling@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "GuidedShelterLoopFunc.h"

/****************************************/
/****************************************/

GuidedShelterLoopFunc::GuidedShelterLoopFunc() {
  m_fRadius = 0.3;
  m_cCoordBlackSpot = CVector2(0.5,0);
  m_cCoordWhiteSpot = CVector2(-0.5, 0);
  m_unScoreSpot = 0;
  m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

GuidedShelterLoopFunc::GuidedShelterLoopFunc(const GuidedShelterLoopFunc& orig) {}

/****************************************/
/****************************************/

GuidedShelterLoopFunc::~GuidedShelterLoopFunc() {}

/****************************************/
/****************************************/

void GuidedShelterLoopFunc::Destroy() {}

void GuidedShelterLoopFunc::Init(TConfigurationNode& t_tree) {
    TConfigurationNode cParametersNode;
    // Use the configuration node to set up the shelter walls
    try {
        cParametersNode = GetNode(t_tree, "params");
        GetNodeAttributeOrDefault(cParametersNode, "w", m_fWidth, (Real) 0);
        GetNodeAttributeOrDefault(cParametersNode, "d", m_fDepth, (Real) 0);
    } catch(std::exception e) {
        LOGERR << e.what() << std::endl;
    }
    CSpace::TMapPerType& tBoxMap = GetSpace().GetEntitiesByType("box");
    for (CSpace::TMapPerType::iterator it = tBoxMap.begin(); it != tBoxMap.end(); ++it) {
        CBoxEntity *pcBox = any_cast<CBoxEntity *>(it->second);
        if(MatchPattern(pcBox->GetId(), "shelter_wall")) {
            CVector3 newBoxSize(0.01, m_fDepth, 0.08);
            pcBox->SetSize(newBoxSize); // TODO: Check if this breaks the bounding box

            //TODO: At the moment the box occupies half of its width in the target zone. Maybe we need to change that.
            //TODO: For that, change the yPos to also respect the size of the pcBox
            CVector3 newBoxPosition(0,0,0);
            Real yPos = -(1.231 - m_fDepth/2);
            if(MatchPattern(pcBox->GetId(), "shelter_wall_1")) {
                newBoxPosition.Set(m_fWidth/2, yPos, 0);
            }
            else {
                newBoxPosition.Set(-m_fWidth/2, yPos, 0);
            }
            MoveEntity((*pcBox).GetEmbodiedEntity(), newBoxPosition, pcBox->GetEmbodiedEntity().GetOriginAnchor().Orientation);
        }
    }
    // Get the light position once
    GetLightPosition();
    CoreLoopFunctions::Init(t_tree);
}

void GuidedShelterLoopFunc::GetLightPosition() {
    // get the position of the light
    CSpace::TMapPerType& tLightMap = GetSpace().GetEntitiesByType("light");
    CVector2 m_cLightPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tLightMap.begin(); it != tLightMap.end(); ++it) {
        CLightEntity *pcLight = any_cast<CLightEntity*>(it->second);
        m_cLightPosition.Set(pcLight->GetPosition().GetX(),
                           pcLight->GetPosition().GetY());
        // LOG << "Light is at " << m_cLightPosition.GetX() << " " << m_cLightPosition.GetY() << std::endl;
    }
}

/****************************************/
/****************************************/

void GuidedShelterLoopFunc::Reset() {
  m_fObjectiveFunction = 0;
  m_unScoreSpot = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor GuidedShelterLoopFunc::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  //TODO: Check it's in the arena bounds

  Real shelter_y_limit = -(1.231 - m_fDepth);
  Real shelter_x_limit = m_fWidth/2;
  //calculate the black area
  if (vCurrentPoint.GetY() < shelter_y_limit) {
      if ((vCurrentPoint.GetX() > -shelter_x_limit) && (vCurrentPoint.GetX() < shelter_x_limit)) {
        return CColor::BLACK;
      } else {
        return CColor::GRAY50;
      }
  }
  //calculate the white area
  //TODO: Projection towards the light
  if ((vCurrentPoint.GetX() > -shelter_x_limit) && (vCurrentPoint.GetX() < shelter_x_limit)) {
      return CColor::WHITE;
  } else {
      return CColor::GRAY50;
  }
}

/****************************************/
/****************************************/

void GuidedShelterLoopFunc::PostStep() {
    m_unScoreSpot = 0;
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        Real shelter_y_limit = -(1.231 - m_fDepth);
        Real shelter_x_limit = m_fWidth/2;
        //calculate the black area
        if (cEpuckPosition.GetY() < shelter_y_limit) {
            if ((cEpuckPosition.GetX() > -shelter_x_limit) && (cEpuckPosition.GetX() < shelter_x_limit)) {
                m_unScoreSpot++;
                // LOG << "Robot is in goal" << std::endl;
            }
        }
    }
    m_fObjectiveFunction += m_unScoreSpot / (Real) m_unNumberRobots;
}

/****************************************/
/****************************************/

void GuidedShelterLoopFunc::PostExperiment() {
  // TODO: Maybe normalize by the execution time?
  LOG << m_fObjectiveFunction << std::endl;
}


/****************************************/
/****************************************/

Real GuidedShelterLoopFunc::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 GuidedShelterLoopFunc::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(GuidedShelterLoopFunc, "guided_shelter_loopfunc");
