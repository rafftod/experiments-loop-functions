#ifndef EXPLORATION_LOOP_FUNC
#define EXPLORATION_LOOP_FUNC

#include <vector>

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

// Version 2
class ExplorationLoopFunction: public CoreLoopFunctions {
  public:
    ExplorationLoopFunction();
    ExplorationLoopFunction(const ExplorationLoopFunction& orig);
    virtual ~ExplorationLoopFunction();

    virtual void Destroy();
    virtual void Reset();
    virtual void Init(argos::TConfigurationNode& t_tree);

	  virtual void PostStep();
    virtual void PostExperiment();

    Real GetObjectiveFunction();

    /*
     * Returns a vector containing a random position inside a circle of radius
     * m_fDistributionRadius and centered in (0,0).
     */
    virtual CVector3 GetRandomPosition();

  private:
	  virtual Real ComputeStepObjectiveValue();
    virtual void RegisterPositions();
    virtual void InitGrid();
	  Real m_ObjectiveFunction;
    std::vector<bool> grid;
    CVector2 sizeArena;
	Real maxScore;
};

#endif
