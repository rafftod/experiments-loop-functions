#include "GiandujaBeaconStopLoopFunc.h"

/****************************************/
/****************************************/

GiandujaBeaconStopLoopFunction::GiandujaBeaconStopLoopFunction() {
  //m_fRadius = 0.10;
  // m_cCoordSpot1 = CVector2(0.6,0.8);
  m_CCoordRect1Pos = CVector2(0.8,-0.5);
  // m_CCoordRect1 = CVector2(1.25,-0.5);
  //
  m_CCoordRect2Pos = CVector2(-0.8,-1);
  // m_CCoordRect2 = CVector2(-1.25,-1.25);

  m_unCostI = 0;
  m_unCostO = 0;
  m_unState = 0;
  m_unTbar = 0;
  m_fObjectiveFunction = 0;
  m_unTime = 0;
}

/****************************************/
/****************************************/

GiandujaBeaconStopLoopFunction::GiandujaBeaconStopLoopFunction(const GiandujaBeaconStopLoopFunction& orig) {}

/****************************************/
/****************************************/

GiandujaBeaconStopLoopFunction::~GiandujaBeaconStopLoopFunction() {}

/****************************************/
/****************************************/

void GiandujaBeaconStopLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void GiandujaBeaconStopLoopFunction::Reset() {
    CoreLoopFunctions::Reset();
    m_unCostI = 0;
    m_unCostO = 0;
    m_unState = 0;
    m_unTbar = 0;
    m_unTime = 0;
    m_fObjectiveFunction = 0;
    ExtractTime();
    PlaceBeacon();
    //Real a =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //Real b =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);
}

/****************************************/
/****************************************/

void GiandujaBeaconStopLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    ExtractTime();
    PlaceBeacon();
    //Real a =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //Real b =m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    //m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);
    //LOG << "Spot is at :" << m_cCoordSpot1 << std::endl;
}

argos::CColor GiandujaBeaconStopLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  //Real d = (m_cCoordSpot1 - vCurrentPoint).Length();

  // if (d <= m_fRadius) {
  //   return CColor::BLACK;
  // }

  // if ( (vCurrentPoint.GetX()<=m_CCoordRect1Pos.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2Pos.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2Pos.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1Pos.GetY()) ) {
  //   return CColor::GREEN;
  // }

  // if ( (vCurrentPoint.GetX()<=m_CCoordRect1.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1.GetY()) ) {
  //   return CColor::WHITE;
  // }

  return CColor::GRAY50;
}


void GiandujaBeaconStopLoopFunction::ExtractTime() {
    try {
        CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        CEPuckBeacon& cController = dynamic_cast<CEPuckBeacon&>(cEntity.GetControllableEntity().GetController());
        m_unTbar = cController.getTBar();
        LOG << "Time=" << m_unTbar << std::endl;
    } catch (std::exception& ex) {
        LOGERR << "Error while casting ExtractTime: " << ex.what() << std::endl;
    }
}


void GiandujaBeaconStopLoopFunction::PlaceBeacon() {
    try {
        CEPuckEntity& cEpuck = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        MoveEntity(cEpuck.GetEmbodiedEntity(),
                         CVector3(0, 0.5, 0),
                         CQuaternion().FromEulerAngles(CRadians::ZERO,
                         CRadians::ZERO,CRadians::ZERO),false);
     } catch (std::exception& ex) {
         LOGERR << "Error while casting PlaceBeacon: " << ex.what() << std::endl;
     }

}


CVector3 GiandujaBeaconStopLoopFunction::GetRandomPosition() {
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

void GiandujaBeaconStopLoopFunction::PostStep() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    UInt8 un_trigger = 0;
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        //Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();
        if (m_unState == 0) {
            if (m_unTime > m_unTbar) {
                un_trigger = 1;
            }
            else if ( fabs(m_tOldPosPoints[pcEpuck].GetX() - cEpuckPosition.GetX()) < 0.0005 && fabs(m_tOldPosPoints[pcEpuck].GetY() - cEpuckPosition.GetY()) < 0.0005) {
                m_unCostI+=1;
            }
        }
        else if (m_unState == 1) {
            if ( fabs(m_tOldPosPoints[pcEpuck].GetX() - cEpuckPosition.GetX()) > 0.0005 && fabs(m_tOldPosPoints[pcEpuck].GetY() - cEpuckPosition.GetY()) > 0.0005) {
                m_unCostO+=1;
            }
        }
        m_tOldPosPoints[pcEpuck] = cEpuckPosition;
    }

    if (m_unState == 0 && un_trigger == 1) {
        m_unState = 1;
        LOG << "time triggered." << std::endl;
    }
    m_unTime+=1;
}

/****************************************/
/****************************************/

void GiandujaBeaconStopLoopFunction::PostExperiment() {
    LOG<< "CostI :" << m_unCostI << " / CostO :" << m_unCostO << " / Tbar:" << m_unTbar << std::endl;
    LOG<< 24000 - (m_unCostI + m_unCostO) << std::endl;
    m_fObjectiveFunction = (Real) 24000 - (m_unCostI + m_unCostO);

}

Real GiandujaBeaconStopLoopFunction::GetObjectiveFunction() {
  return (m_fObjectiveFunction);
}

REGISTER_LOOP_FUNCTIONS(GiandujaBeaconStopLoopFunction, "gianduja_beacon_stop_loop_functions");
