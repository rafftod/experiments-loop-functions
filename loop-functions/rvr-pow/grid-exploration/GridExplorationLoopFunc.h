#ifndef GRID_EXPLORATION_LOOP_FUNC
#define GRID_EXPLORATION_LOOP_FUNC

#include "../../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/rvr/simulator/rvr_entity.h>

using namespace argos;

class GridExplorationLoopFunction : public CoreLoopFunctions
{
public:
  GridExplorationLoopFunction();
  GridExplorationLoopFunction(const GridExplorationLoopFunction &orig);
  virtual ~GridExplorationLoopFunction();

  virtual void Destroy();

  virtual argos::CColor GetFloorColor(const argos::CVector2 &c_position_on_plane);
  virtual void PostExperiment();
  virtual void PostStep();
  virtual void Init(TConfigurationNode &t_tree);
  virtual void Reset();
  virtual CVector3 GetRandomPosition();

  Real GetObjectiveFunction();

  void ArrestTrespassers();
  CVector3 GetJailPosition();

private:
  std::vector<std::vector<int>> m_grid;

  Real m_arenaSize;
  UInt32 m_gridSize;
  Real m_fObjectiveFunction;
};

#endif
