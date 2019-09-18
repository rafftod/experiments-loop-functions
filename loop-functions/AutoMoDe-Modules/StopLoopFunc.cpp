#include "StopLoopFunc.h"

/****************************************/
// Version 2
/****************************************/

StopLoopFunction::StopLoopFunction() {
  m_ObjectiveFunction = 0;
}

/****************************************/
/****************************************/

void StopLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
  previous.reserve(m_unNumberRobots);
  int count = 0;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    previous.push_back(new CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY())); 
    count++;
  }
}

/****************************************/
/****************************************/

StopLoopFunction::StopLoopFunction(const StopLoopFunction& orig) {}

/****************************************/
/****************************************/

StopLoopFunction::~StopLoopFunction() {}

/****************************************/
/****************************************/

void StopLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void StopLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void StopLoopFunction::PostStep() {
	m_ObjectiveFunction += ComputeStepObjectiveValue();  
}

/****************************************/
/****************************************/

Real StopLoopFunction::ComputeStepObjectiveValue() {
  Real temp0 = 0;
  int count = 0;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    Real temp = (*(new CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY())) - *previous[count]).Length();
    previous[count]->Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    temp0 += temp * 100;
    count++;
  } 
  return temp0;
}

/****************************************/
/****************************************/

void StopLoopFunction::PostExperiment() {
  LOG << "Stop" << std::endl;
  m_ObjectiveFunction *= -1;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real StopLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 StopLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(StopLoopFunction, "stop_loop_functions");
