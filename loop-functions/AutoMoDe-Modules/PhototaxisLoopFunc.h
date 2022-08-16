#ifndef PHOTOTAXIS_LOOP_FUNC
#define PHOTOTAXIS_LOOP_FUNC

#include <cmath>

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

using namespace argos;

// Version 2
class PhototaxisLoopFunction: public CoreLoopFunctions {
  public:
    PhototaxisLoopFunction();
    PhototaxisLoopFunction(const PhototaxisLoopFunction& orig);
    virtual ~PhototaxisLoopFunction();

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
    CVector2 sizeArena;
    CVector2 lightPosition;
};

#endif
