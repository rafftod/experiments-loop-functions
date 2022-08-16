/**
  * @file <loop-functions/example/GuidedShelter.h>
  *
  * @author Jonas Kuckling - <jonas.kuckling@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef EXAMPLE_AGGREGATION_LOOP_FUNC
#define EXAMPLE_AGGREGATION_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/utility/string_utilities.h>

using namespace argos;

class GuidedShelterLoopFunc: public CoreLoopFunctions {
  public:
    GuidedShelterLoopFunc();
    GuidedShelterLoopFunc(const GuidedShelterLoopFunc& orig);
    virtual ~GuidedShelterLoopFunc();

    virtual void Destroy();
    virtual void Reset();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void Init(TConfigurationNode& t_tree);
    virtual void PostStep();
    virtual void PostExperiment();

    Real GetObjectiveFunction();

    /*
     * Returns a vector containing a random position inside a circle of radius
     * m_fDistributionRadius and centered in (0,0).
     */
    virtual CVector3 GetRandomPosition();

  private:

    void GetLightPosition();
    bool PointIsInBlackArea(CVector2 point);
    bool PointIsInWhiteArea(CVector2 point);

    Real m_fDepth;
    Real m_fWidth;
    Real m_fOffset;
    CVector2 m_cLightPosition;
    // I forgot why I put the following line but it broke on my system. Maybe it was used on the cluster?
    // static const Real ARENA_DEPTH = 1.231;
    // The current version is only const. Seems to work on my computer
    const Real ARENA_DEPTH = 1.231;

    Real m_fRadius;
    CVector2 m_cCoordBlackSpot;
    CVector2 m_cCoordWhiteSpot;

    UInt32 m_unScoreSpot;
    Real m_fObjectiveFunction;
};

#endif
