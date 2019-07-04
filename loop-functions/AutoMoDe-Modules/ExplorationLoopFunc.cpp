#include "ExplorationLoopFunc.h"
#include <iostream>
#include <fstream>

/****************************************/
// Version 2
/****************************************/

ExplorationLoopFunction::ExplorationLoopFunction() {
  m_ObjectiveFunction = 0;
  maxScore = 1.0;
  sizeArena.Set(1,1);
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
  InitGrid();
  RegisterPositions();
}

void ExplorationLoopFunction::InitGrid(){
  sizeArena.Set(GetSpace().GetArenaSize().GetX(),GetSpace().GetArenaSize().GetY());
  maxScore = ((int)(sizeArena.GetY()*100*sizeArena.GetX()*100))*1.0;
  grid.reserve((unsigned int)maxScore);
}

/****************************************/
/****************************************/

ExplorationLoopFunction::ExplorationLoopFunction(const ExplorationLoopFunction& orig) {}

/****************************************/
/****************************************/

ExplorationLoopFunction::~ExplorationLoopFunction() {}

/****************************************/
/****************************************/

void ExplorationLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ExplorationLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::PostStep() {
  RegisterPositions();
	m_ObjectiveFunction += ComputeStepObjectiveValue(); 
}

/****************************************/
/****************************************/

Real ExplorationLoopFunction::ComputeStepObjectiveValue() {
  Real temp = 0;
  for (unsigned int i = 0; i<(unsigned int)(sizeArena.GetX()*100*sizeArena.GetY()*100); i++) {
    if (grid[i]==true){
      temp+=1;
    }
  }
  return temp/maxScore;
}

void ExplorationLoopFunction::RegisterPositions() {
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    unsigned int x = (unsigned int)((pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX()+sizeArena.GetX()/2.0)*100);
    unsigned int y = (unsigned int)((pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()+sizeArena.GetY()/2.0)*100);
    grid[(unsigned int)(x*sizeArena.GetX()*100+y)] = true;
  }
}

/****************************************/
/****************************************/

void ExplorationLoopFunction::PostExperiment() {
  LOG << "Exploration" << std::endl;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real ExplorationLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 ExplorationLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(ExplorationLoopFunction, "exploration_loop_functions");
