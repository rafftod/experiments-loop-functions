/**
  *
  * @author Ken H - <khasselm@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef DIRECTIONAL_GATE
#define DIRECTIONAL_GATE

#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "../../src/CoreLoopFunctions.h"

using namespace argos;

class DirectionalGate: public CoreLoopFunctions {
  public:
    DirectionalGate();
    DirectionalGate(const DirectionalGate& orig);
    virtual ~DirectionalGate();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();
    virtual void Reset();

    Real GetObjectiveFunction();

    CVector3 GetRandomPosition();

  private:
    typedef std::map<CEPuckEntity*, CVector2 > TOldPosMap;
    TOldPosMap m_tOldPosPoints;
    Real m_fObjectiveFunction;
    CVector2 m_RectCoord1;
    CVector2 m_RectCoord2;
    Real m_fHeightGateZone;
    Real m_fWidthShelter;
    Real m_fHeightShelter;
    Real m_fWidthWallsShelter;
    Real m_fHeightWallsShelter;
    CVector2 m_cPositionShelter;
    CBoxEntity *m_pcBoxLeft, *m_pcBoxRight;
};

#endif
