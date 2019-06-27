#include "AttractionLoopFunc.h"

/****************************************/
// Version 2
/****************************************/

AttractionLoopFunction::AttractionLoopFunction() {
  m_ObjectiveFunction = 0;
  sizeArena = CVector2(0,0);
  nbrCases = 0;
}

/****************************************/
/****************************************/

void AttractionLoopFunction::Init(argos::TConfigurationNode& t_tree){
  CoreLoopFunctions::Init(t_tree);
  sizeArena.Set(GetSpace().GetArenaSize().GetX(),GetSpace().GetArenaSize().GetY());
  std::cout << sizeArena.GetX() << ":" << sizeArena.GetY() << std::endl;
  nbrCases = m_unNumberRobots * (m_unNumberRobots-1) / 2;
}

/****************************************/
/****************************************/

AttractionLoopFunction::AttractionLoopFunction(const AttractionLoopFunction& orig) {}

/****************************************/
/****************************************/

AttractionLoopFunction::~AttractionLoopFunction() {}

/****************************************/
/****************************************/

void AttractionLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void AttractionLoopFunction::Reset() {
  m_ObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

void AttractionLoopFunction::PostStep() {
	m_ObjectiveFunction += ComputeStepObjectiveValue();  
}

/****************************************/
/****************************************/

Real AttractionLoopFunction::ComputeStepObjectiveValue() {
  Real temp = 0;
  std::vector<CVector2*> positions;
  positions.reserve(m_unNumberRobots);
  CSpace::TMapPerType& tEpuckMap = GetSpace().GetEntitiesByType("epuck");
  for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it) {
    CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*>(it->second);
    positions.push_back(new CVector2(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(), pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY()));
  }
  for (unsigned int i = 0; i<m_unNumberRobots-1; i++){
    for (unsigned int j = i+1; j<m_unNumberRobots; j++){
      temp += (*positions[i] - *positions[j]).Length();
    }
  }
  return temp;
}

/****************************************/
/****************************************/

void AttractionLoopFunction::PostExperiment() {
  LOG << "Attraction" << std::endl;
  m_ObjectiveFunction *= -1;
  LOG << "Objective function result = " << m_ObjectiveFunction << std::endl;
}

/****************************************/
/****************************************/

Real AttractionLoopFunction::GetObjectiveFunction() {
  return m_ObjectiveFunction;
}

/****************************************/
/****************************************/

CVector3 AttractionLoopFunction::GetRandomPosition() {
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

REGISTER_LOOP_FUNCTIONS(AttractionLoopFunction, "attraction_loop_functions");
