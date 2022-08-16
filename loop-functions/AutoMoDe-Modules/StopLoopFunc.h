#ifndef STOP_LOOP_FUNC
#define STOP_LOOP_FUNC

#include <vector>

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

// Version 2
class StopLoopFunction: public CoreLoopFunctions {
  public:
    StopLoopFunction();
    StopLoopFunction(const StopLoopFunction& orig);
    virtual ~StopLoopFunction();

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
	  std::vector<CVector2*> previous;
	  Real m_ObjectiveFunction;
};

#endif
