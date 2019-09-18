#include "RepulsionLoopFunc.h"

/****************************************/
// Version 2
/****************************************/

RepulsionLoopFunction::RepulsionLoopFunction() {
  m_ObjectiveFunction = 0;
}

/****************************************/
/****************************************/

void RepulsionLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
}

/****************************************/
/****************************************/

RepulsionLoopFunction::RepulsionLoopFunction(const RepulsionLoopFunction& orig) {}

/****************************************/
/****************************************/

RepulsionLoopFunction::~RepulsionLoopFunction() {}

/****************************************/
/****************************************/

void RepulsionLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void RepulsionLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void RepulsionLoopFunction::PostStep() {
	m_ObjectiveFunction += ComputeStepObjectiveValue();  
}

/****************************************/
/****************************************/

Real RepulsionLoopFunction::ComputeStepObjectiveValue() {
  Real temp = 0;
  std::vector<CVector2*> positions;
  positions.reserve(m_unNumberRobots);
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    positions.push_back(new CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()));
  }
  for (unsigned int i = 0; i<m_unNumberRobots-1; i++){
    Real distance = 0;
    Real temp2 = -1; // "Empty" value, to indicate to take the new distance anyways
    for (unsigned int j = 0; j<m_unNumberRobots; j++){
      if(i!=j){
        distance = (*positions[i] - *positions[j]).Length();
        temp2 = (temp2<distance && temp2!=-1)?temp2:distance;
      }
    }
    temp += temp2;
  }
  return temp;
}

/****************************************/
/****************************************/

void RepulsionLoopFunction::PostExperiment() {
  LOG << "Repulsion" << std::endl;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real RepulsionLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 RepulsionLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(RepulsionLoopFunction, "repulsion_loop_functions");
