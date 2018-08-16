#include "GiandujaBeaconAggregationLoopFunc.h"

/****************************************/
/****************************************/

GiandujaBeaconAggregationLoopFunction::GiandujaBeaconAggregationLoopFunction() {
  m_fRadius = 0.3;
  m_cCoordSpot1 = CVector2(0.5,0.5);
  m_cCoordSpot2 = CVector2(-0.5,0.5);
  m_unCostSpot1 = 0;
  m_fObjectiveFunction = 0;
  m_Tbar = 0;
  m_State = 0;
  m_unTime = 0;
}

/****************************************/
/****************************************/

GiandujaBeaconAggregationLoopFunction::GiandujaBeaconAggregationLoopFunction(const GiandujaBeaconAggregationLoopFunction& orig) {}

/****************************************/
/****************************************/

GiandujaBeaconAggregationLoopFunction::~GiandujaBeaconAggregationLoopFunction() {}

/****************************************/
/****************************************/

void GiandujaBeaconAggregationLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void GiandujaBeaconAggregationLoopFunction::Reset() {
    CoreLoopFunctions::Reset();
    m_unCostSpot1 = 0;
    m_fObjectiveFunction = 0;
    m_State = 0;
    m_Tbar = 0;
    m_unTime = 0;
    ExtractTime();
    PlaceBeacon();
    //PlaceLight();
}


void GiandujaBeaconAggregationLoopFunction::Init(TConfigurationNode& t_tree) {
    CoreLoopFunctions::Init(t_tree);
    //PlaceLight();
    ExtractTime();
    PlaceBeacon();
}


/****************************************/
/****************************************/

argos::CColor GiandujaBeaconAggregationLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
  Real d = (m_cCoordSpot1 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::WHITE;
  }

  d = (m_cCoordSpot2 - vCurrentPoint).Length();
  if (d <= m_fRadius) {
    return CColor::BLACK;
  }

  return CColor::GRAY50;
}


void GiandujaBeaconAggregationLoopFunction::ExtractTime() {
    try {
        CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity("beacon0"));
        CEPuckBeacon& cController = dynamic_cast<CEPuckBeacon&>(cEntity.GetControllableEntity().GetController());
        m_Tbar = cController.getTBar();
        LOG << "Time=" << m_Tbar << std::endl;
    } catch (std::exception& ex) {
        LOGERR << "Error while casting ExtractTime: " << ex.what() << std::endl;
    }
}


void GiandujaBeaconAggregationLoopFunction::PlaceBeacon() {
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

// void GiandujaBeaconAggregationLoopFunction::PlaceLight() {
//     // /* Place a light at a random spot to prevent robots to exploit (anti)phototaxis behaviours as RW */
//     CLightEntity& cLight = dynamic_cast<CLightEntity&>(GetSpace().GetEntity("light0"));
//     /* Consider the light only if it has non zero intensity */
//     cLight.SetIntensity(m_pcRng->Uniform(CRange<Real>(0.05f, 0.8f)));
//
//     Real ab;
//     Real DistributionRadius = 1.3;
//
//     ab = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
//     Real posX = DistributionRadius * cos(2 * CRadians::PI.GetValue() * ab);
//     Real posY = DistributionRadius * sin(2 * CRadians::PI.GetValue() * ab);
//
//     cLight.MoveTo(CVector3(posX, posY, 0.4),CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
//     CRadians::ZERO,CRadians::ZERO));
// }

CVector3 GiandujaBeaconAggregationLoopFunction::GetRandomPosition() {
    Real a;
    Real b;

    a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosY = -1.0 + a * 0.8;
    Real fPosX = -0.8 + b * 1.6;

    return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

void GiandujaBeaconAggregationLoopFunction::PostStep() {
    CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);

    if (m_unTime > m_Tbar) {
        m_State = 1;
    }

    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
        CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                         pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        Real fDistanceSpot1 = (m_cCoordSpot1 - cEpuckPosition).Length();

        if (m_State == 0 && fDistanceSpot1 <= m_fRadius) {
            m_unCostSpot1 += 2;
        }
        else if (m_State == 1 && fDistanceSpot1 >= m_fRadius) {
            m_unCostSpot1 += 2;
        }
    }
    m_fObjectiveFunction = (Real) m_unCostSpot1;
    m_unTime+=1;
}

/****************************************/
/****************************************/

void GiandujaBeaconAggregationLoopFunction::PostExperiment() {
    LOG<< "fit: " << 24000-m_fObjectiveFunction << std::endl;
}

Real GiandujaBeaconAggregationLoopFunction::GetObjectiveFunction() {
    SInt32 score = 24000-m_fObjectiveFunction;
    if (score < 0) {
        score = 0;
    }
    return (score);
}

REGISTER_LOOP_FUNCTIONS(GiandujaBeaconAggregationLoopFunction, "gianduja_beacon_aggregation_loop_functions");
