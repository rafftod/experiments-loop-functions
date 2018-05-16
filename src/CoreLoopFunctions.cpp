/*
 * @file <src/CoreLoopFunctions.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 * @author Ken Hasselmann - <khasselm@ulb.ac.be>
 *
 * @package experiments-loop-functions
 *
 * @license MIT License
 */


#include "CoreLoopFunctions.h"

/****************************************/
/****************************************/

void CoreLoopFunctions::Init(argos::TConfigurationNode& t_tree) {
  m_pcRng = CRandom::CreateRNG("argos");
  TConfigurationNode cParametersNode;
  try {
    cParametersNode = GetNode(t_tree, "params");
    GetNodeAttributeOrDefault(cParametersNode, "number_robots", m_unNumberRobots, (UInt32) 1);
    GetNodeAttributeOrDefault(cParametersNode, "dist_radius", m_fDistributionRadius, (Real) 0);
    GetNodeAttributeOrDefault(cParametersNode, "controller", m_strControllerId, std::string("controller_not_specified"));
  } catch(std::exception e) {
    LOGERR << e.what() << std::endl;
  }

  MoveRobots();
}

/****************************************/
/****************************************/

void CoreLoopFunctions::Reset() {
  MoveRobots();
}

/****************************************/
/****************************************/

CoreLoopFunctions::~CoreLoopFunctions() {}

/****************************************/
/****************************************/

// This function is decrepated: the robots should be instanciated in the .argos file
void CoreLoopFunctions::PositionRobots() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  LOG << "[LF INFO] Launching control software \"" << m_strControllerId << "\"" << std::endl;
  for(UInt32 i = 1; i < m_unNumberRobots + 1; ++i) {
    std::ostringstream id;
    id << "epuck" << i;
    pcEpuck = new CEPuckEntity(id.str().c_str(),
                               m_strControllerId,
                               CVector3(0,0,0),
                               CQuaternion().FromEulerAngles(CRadians::ZERO,CRadians::ZERO,CRadians::ZERO));
    AddEntity(*pcEpuck);
    // Choose position at random
    unTrials = 0;
    do {
       ++unTrials;
       CVector3 cEpuckPosition = GetRandomPosition();
       bPlaced = MoveEntity((*pcEpuck).GetEmbodiedEntity(),
                            cEpuckPosition,
                            CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                            CRadians::ZERO,CRadians::ZERO),false);
    } while(!bPlaced && unTrials < 100);
    if(!bPlaced) {
       THROW_ARGOSEXCEPTION("Can't place robot");
    }
  }
}

/****************************************/
/****************************************/

void CoreLoopFunctions::MoveRobots() {
  CEPuckEntity* pcEpuck;
  bool bPlaced = false;
  UInt32 unTrials;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    pcEpuck = any_cast<CEPuckEntity*>(it->second);
    // Using the controller specified in the params of the loop functions
    pcEpuck->GetControllableEntity().SetController(m_strControllerId);
    // Choose position at random
    unTrials = 0;
    do {
       ++unTrials;
       CVector3 cEpuckPosition = GetRandomPosition();
       bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                            cEpuckPosition,
                            CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO,CRadians::TWO_PI)),
                            CRadians::ZERO,CRadians::ZERO),false);
    } while(!bPlaced && unTrials < 100);
    if(!bPlaced) {
       THROW_ARGOSEXCEPTION("Can't place robot");
    }
  }
}

/****************************************/
/****************************************/

void CoreLoopFunctions::RemoveRobots() {
  for(UInt32 i = 1; i < m_unNumberRobots + 1; ++i) {
    std::ostringstream id;
    id << "epuck" << i;
    RemoveEntity(id.str().c_str());
  }
}
