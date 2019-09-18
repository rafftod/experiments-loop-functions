#ifndef REPULSION_LOOP_FUNC
#define REPULSION_LOOP_FUNC

#include <vector>

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

// Version 2
class RepulsionLoopFunction: public CoreLoopFunctions {
  public:
    RepulsionLoopFunction();
    RepulsionLoopFunction(const RepulsionLoopFunction& orig);
    virtual ~RepulsionLoopFunction();

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
    Real m_ObjectiveFunction;
};

#endif
