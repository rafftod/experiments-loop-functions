#include "GiandujaBeaconDecisionLoopFunc.h"

/****************************************/
/****************************************/

GiandujaBeaconDecisionLoopFunction::GiandujaBeaconDecisionLoopFunction() {
  m_CCoordRect1Pos = CVector2(0.8,1);
  m_CCoordRect2Pos = CVector2(-0.8,-0.51);
  m_CCoordLimit = CVector2(-0.5, -0.5);

  m_unCostI = 0;
  m_unState = 0;

  m_unTbar = 600;
  m_unTbar2 = 800;

  m_fObjectiveFunction = 0;
  m_unTime = 0;
}

/****************************************/
/****************************************/

GiandujaBeaconDecisionLoopFunction::GiandujaBeaconDecisionLoopFunction(const GiandujaBeaconDecisionLoopFunction& orig) {}

/****************************************/
/****************************************/

GiandujaBeaconDecisionLoopFunction::~GiandujaBeaconDecisionLoopFunction() {}

/****************************************/
/****************************************/

void GiandujaBeaconDecisionLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void GiandujaBeaconDecisionLoopFunction::Reset() {
    CoreLoopFunctions::Reset();
    m_unCostI = 0;
    m_unState = 0;
    m_unTime = 0;
    m_fObjectiveFunction = 0;
    m_unMessage = 0;
    PlaceBeacon();
    ExtractMessage();
    //Real a =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //Real b =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);
}

/****************************************/
/****************************************/

void GiandujaBeaconDecisionLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    PlaceBeacon();
    ExtractMessage();
    //Real a =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //Real b =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);
    //LOG << "Spot is at :" << m_cCoordSpot1 << std::endl;
}

argos::CColor GiandujaBeaconDecisionLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  //Real d = (m_cCoordSpot1 - vCurrentPoint).Length();

  if ( vCurrentPoint.GetY() <= m_CCoordLimit.GetY() ) {
    return CColor::BLACK;
  }

  // if ( (vCurrentPoint.GetX()<=m_CCoordRect1Pos.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2Pos.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2Pos.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1Pos.GetY()) ) {
  //   return CColor::GREEN;
  // }

  // if ( (vCurrentPoint.GetX()<=m_CCoordRect1.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1.GetY()) ) {
  //   return CColor::WHITE;
  // }

  return CColor::GRAY50;
}

void GiandujaBeaconDecisionLoopFunction::ExtractMessage() {
    try {
        CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        CEPuckBeacon& cController = dynamic_cast<CEPuckBeacon&>(cEntity.GetControllableEntity().GetController());
        m_unMessage = cController.getMessage();
    } catch (std::exception& ex) {
        LOGERR << "Error while casting ExtractMessage: " << ex.what() << std::endl;
    }
}


void GiandujaBeaconDecisionLoopFunction::PlaceBeacon() {
    try {
        CEPuckEntity& cEpuck = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        MoveEntity(cEpuck.GetEmbodiedEntity(),
                         CVector3(0, -0.5, 0),
                         CQuaternion().FromEulerAngles(CRadians::ZERO,
                         CRadians::ZERO,CRadians::ZERO),false);
     } catch (std::exception& ex) {
         LOGERR << "Error while casting PlaceBeacon: " << ex.what() << std::endl;
     }

}

void GiandujaBeaconDecisionLoopFunction::ChangeMessage() {
    try {
        CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        CEPuckBeacon& cController = dynamic_cast<CEPuckBeacon&>(cEntity.GetControllableEntity().GetController());

        if (m_unMessage == 10) {
            m_unMessage = 160;
        }
        else if (m_unMessage == 160) {
            m_unMessage = 10;
        }

        cController.setMessage(m_unMessage);
    } catch (std::exception& ex) {
        LOGERR << "Error while casting ChangeMessage: " << ex.what() << std::endl;
    }
}


CVector3 GiandujaBeaconDecisionLoopFunction::GetRandomPosition() {
    Real a;
    Real b;

    a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = m_CCoordRect2Pos.GetX() + a*fabs(m_CCoordRect2Pos.GetX() - m_CCoordRect1Pos.GetX());
    Real fPosY = m_CCoordRect2Pos.GetY() + b*fabs(m_CCoordRect2Pos.GetY() - m_CCoordRect1Pos.GetY());

    return CVector3(fPosX, fPosY, 0);
}


/****************************************/
/****************************************/

void GiandujaBeaconDecisionLoopFunction::PostStep() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        if (m_unMessage == 160) {
            if ( cEpuckPosition.GetY() > m_CCoordLimit.GetY() ) {
                m_unCostI+=1;
            }
        }
        else if (m_unMessage == 10) {
            if ( cEpuckPosition.GetY() <= m_CCoordLimit.GetY() ) {
                m_unCostI+=1;
            }
        }

    }


    if (m_unState == 0 && m_unTime > m_unTbar) {
        ChangeMessage();
        m_unState = 1;
        LOG << "time1 triggered." << std::endl;
    }
    // else if (m_unState == 1 && m_unTime > m_unTbar2) {
    //     ChangeMessage();
    //     m_unState = 2;
    //     LOG << "time2 triggered." << std::endl;
    // }
    m_unTime+=1;
}

/****************************************/
/****************************************/

void GiandujaBeaconDecisionLoopFunction::PostExperiment() {
    LOG<< 24000 - (m_unCostI) << std::endl;
    m_fObjectiveFunction = (Real) 24000 - (m_unCostI);

}

Real GiandujaBeaconDecisionLoopFunction::GetObjectiveFunction() {
  return (m_fObjectiveFunction);
}

REGISTER_LOOP_FUNCTIONS(GiandujaBeaconDecisionLoopFunction, "gianduja_beacon_decision_loop_functions");
