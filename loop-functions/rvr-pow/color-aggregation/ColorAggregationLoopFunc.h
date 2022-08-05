#ifndef GRID_EXPLORATION_LOOP_FUNC
#define GRID_EXPLORATION_LOOP_FUNC

#include "../../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/rvr/simulator/rvr_entity.h>
#include <tr1/unordered_map>

using namespace argos;

class ColorAggregationLoopFunction : public CoreLoopFunctions
{
public:
    ColorAggregationLoopFunction();
    ColorAggregationLoopFunction(const ColorAggregationLoopFunction &orig);
    virtual ~ColorAggregationLoopFunction();

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
    Real m_patchesSize;
    Real m_fObjectiveFunction;
    // std::tr1::unordered_map<CColor, std::vector<std::pair<Real, Real>>> m_colorMap;
    std::vector<std::vector<std::pair<Real, Real>>> m_colorMap;

    virtual void CreateColorTiles();

    virtual CColor GetColorParameter(const UInt32 &un_value);
};

#endif
