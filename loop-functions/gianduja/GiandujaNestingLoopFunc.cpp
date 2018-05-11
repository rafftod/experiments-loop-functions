#include "GiandujaNestingLoopFunc.h"

/****************************************/
/****************************************/

GiandujaNestingLoopFunction::GiandujaNestingLoopFunction() {
  m_fRadius = 0.1;
  m_cCoordSpot1 = CVector2(0.6,0.8);
  m_cCoordSpot2 = CVector2(-0.5,0.5);

  m_CCoordRect1Pos = CVector2(0.8,-0.5);
  m_CCoordRect1 = CVector2(1.25,-0.5);

  m_CCoordRect2Pos = CVector2(-0.8,-1);
  m_CCoordRect2 = CVector2(-1.25,-1.25);


  m_unCostI = 0;
  m_unCostO = 0;
  m_unState = 0;
  m_unTbar = 0;
  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

GiandujaNestingLoopFunction::GiandujaNestingLoopFunction(const GiandujaNestingLoopFunction& orig) {}

/****************************************/
/****************************************/

GiandujaNestingLoopFunction::~GiandujaNestingLoopFunction() {}

/****************************************/
/****************************************/

void GiandujaNestingLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void GiandujaNestingLoopFunction::Reset() {
    CoreLoopFunctions::Reset();
    m_unCostI = 0;
    m_unCostO = 0;
    m_unState = 0;
    m_unTbar = 0;
    m_fObjectiveFunction = 0;
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);
}

/****************************************/
/****************************************/
void GiandujaNestingLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    m_cCoordSpot1 = CVector2(-0.7+a*1.4,0.6+b*0.4);

}

argos::CColor GiandujaNestingLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();

  if (d <= m_fRadius) {
    return CColor::BLACK;
  }
  if (d <= m_fRadius+0.015) {
    return CColor::GREEN;
  }

  // if ( (vCurrentPoint.GetX()<=m_CCoordRect1Pos.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2Pos.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2Pos.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1Pos.GetY()) ) {
  //   return CColor::GREEN;
  // }

  if ( (vCurrentPoint.GetX()<=m_CCoordRect1.GetX()) && (vCurrentPoint.GetX()>=m_CCoordRect2.GetX()) && (vCurrentPoint.GetY()>=m_CCoordRect2.GetY()) && (vCurrentPoint.GetY()<=m_CCoordRect1.GetY()) ) {
    return CColor::WHITE;
  }

  return CColor::GRAY50;
}


CVector3 GiandujaNestingLoopFunction::GetRandomPosition() {
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

void GiandujaNestingLoopFunction::PostStep() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    UInt8 un_trigger = 0;
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();

        if (m_unState == 0) {
            if (fDistanceSpot1 <= m_fRadius+0.015) {
                un_trigger = 1;
            }
            else if ( (cEpuckPosition.GetX()<=m_CCoordRect1.GetX()) && (cEpuckPosition.GetX()>=m_CCoordRect2.GetX()) && (cEpuckPosition.GetY()>=m_CCoordRect2.GetY()) && (cEpuckPosition.GetY()<=m_CCoordRect1.GetY()) ) { //inside nest
                m_unCostI+=1;
            }
            m_unTbar +=1;
        }
        else if (m_unState == 1) {
            if ( !((cEpuckPosition.GetX()<=m_CCoordRect1.GetX()) && (cEpuckPosition.GetX()>=m_CCoordRect2.GetX()) && (cEpuckPosition.GetY()>=m_CCoordRect2.GetY()) && (cEpuckPosition.GetY()<=m_CCoordRect1.GetY())) ) { //outside nest
                m_unCostO+=1;
            }
        }
    }

    if (m_unState == 0 && un_trigger == 1) {
        m_unState = 1;
        LOG << "Found Spot, triggering." << std::endl;
    }

}

/****************************************/
/****************************************/

void GiandujaNestingLoopFunction::PostExperiment() {
    LOG<< "CostI :" << m_unCostI << " / CostO :" << m_unCostO << " / Tbar:" << m_unTbar << std::endl;
    LOG<< m_unCostI + m_unCostO + m_unTbar << std::endl;//+ m_unTbar
    m_fObjectiveFunction = (Real) m_unCostI + m_unCostO + m_unTbar;

}

Real GiandujaNestingLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

REGISTER_LOOP_FUNCTIONS(GiandujaNestingLoopFunction, "gianduja_nesting_loop_functions");
