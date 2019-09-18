#include "AntiPhototaxisLoopFunc.h"

/****************************************/
// Version 1
/****************************************/

AntiPhototaxisLoopFunction::AntiPhototaxisLoopFunction() {
  m_ObjectiveFunction = 0;
  lightPosition = CVector2(0,0);
}

/****************************************/
/****************************************/

void AntiPhototaxisLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
  CLightEntity* light = any_cast<CLightEntity*>(GetSpace().GetEntitiesByType("light")["light"]);
  lightPosition = CVector2(light->GetPosition().GetX(), light->GetPosition().GetY());
  std::cout << "light = " << lightPosition.GetX() << "|" << lightPosition.GetY() << std::endl;
}

/****************************************/
/****************************************/

AntiPhototaxisLoopFunction::AntiPhototaxisLoopFunction(const AntiPhototaxisLoopFunction& orig) {}

/****************************************/
/****************************************/

AntiPhototaxisLoopFunction::~AntiPhototaxisLoopFunction() {}

/****************************************/
/****************************************/

void AntiPhototaxisLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void AntiPhototaxisLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AntiPhototaxisLoopFunction::PostStep() {
	m_ObjectiveFunction += ComputeStepObjectiveValue();  
}

/****************************************/
/****************************************/

Real AntiPhototaxisLoopFunction::ComputeStepObjectiveValue() {
  Real temp = 0;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    temp += (lightPosition - CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY())).Length();
  }
  return temp;
}

/****************************************/
/****************************************/

void AntiPhototaxisLoopFunction::PostExperiment() {
  LOG << "AntiPhototaxis" << std::endl;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AntiPhototaxisLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AntiPhototaxisLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(AntiPhototaxisLoopFunction, "antiphototaxis_loop_functions");
