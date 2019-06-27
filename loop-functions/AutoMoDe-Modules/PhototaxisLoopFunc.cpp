#include "PhototaxisLoopFunc.h"

/****************************************/
// Version 2
/****************************************/

PhototaxisLoopFunction::PhototaxisLoopFunction() {
  m_ObjectiveFunction = 0;
  sizeArena = CVector2(0,0);
  lightPosition = CVector2(0,0);
}

/****************************************/
/****************************************/

void PhototaxisLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
  sizeArena.Set(GetSpace().GetArenaSize().GetX(),GetSpace().GetArenaSize().GetY());
  std::cout << "arena size = "  << sizeArena.GetX() << ":" << sizeArena.GetY() << std::endl;
  CLightEntity* light = any_cast<CLightEntity*>(GetSpace().GetEntitiesByType("light")["light"]);
  lightPosition = CVector2(light->GetPosition().GetX(), light->GetPosition().GetY());
  std::cout << "light = "  << lightPosition.GetX() << "|" << lightPosition.GetY() << std::endl;
}

/****************************************/
/****************************************/

PhototaxisLoopFunction::PhototaxisLoopFunction(const PhototaxisLoopFunction& orig) {}

/****************************************/
/****************************************/

PhototaxisLoopFunction::~PhototaxisLoopFunction() {}

/****************************************/
/****************************************/

void PhototaxisLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void PhototaxisLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void PhototaxisLoopFunction::PostStep() {
	m_ObjectiveFunction += ComputeStepObjectiveValue();  
}

/****************************************/
/****************************************/

Real PhototaxisLoopFunction::ComputeStepObjectiveValue() {
  Real temp = 0;
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    temp += (lightPosition - CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY())).Length();
  }
  return temp * -1.0; //Change: add "* -1.0"
}

/****************************************/
/****************************************/

void PhototaxisLoopFunction::PostExperiment() {
  LOG << "Phototaxis" << std::endl;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real PhototaxisLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 PhototaxisLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(PhototaxisLoopFunction, "phototaxis_loop_functions");
