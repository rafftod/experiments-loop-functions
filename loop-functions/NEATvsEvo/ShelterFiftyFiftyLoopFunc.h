/**
  * @file <loop-functions/example/ShelterFiftyFiftyLoopFunc.h>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#ifndef SHELTER_FIFTY_FIFTY_LOOP_FUNC
#define SHELTER_FIFTY_FIFTY_LOOP_FUNC

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class ShelterFiftyFiftyLoopFunction: public CoreLoopFunctions {
  public:
    ShelterFiftyFiftyLoopFunction();
    ShelterFiftyFiftyLoopFunction(const ShelterFiftyFiftyLoopFunction& orig);
    virtual ~ShelterFiftyFiftyLoopFunction();

    virtual void Destroy();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();

    virtual argos::CColor GetFloorColor(const argos::CVector2& c_position_on_plane);
    virtual void PostExperiment();
    virtual void PostStep();

    Real GetObjectiveFunction();

    /*
     * Returns a vector containing a random position inside a circle of radius
     * m_fDistributionRadius and centered in (0,0).
     */
    virtual CVector3 GetRandomPosition();

  private:
    Real m_fWidthShelterArea;
    Real m_fLengthShelterArea;
    Real m_fDistanceBetweenShelters;

    CRange<Real> m_cBlackFloorX;
    CRange<Real> m_cBlackFloorY;
    CRange<Real> m_cShelterLeftX;
    CRange<Real> m_cShelterLeftY;
    CRange<Real> m_cShelterRightX;
    CRange<Real> m_cShelterRightY;

    CBoxEntity *m_pcBoxTopLeft, *m_pcBoxBottomLeft, *m_pcBoxCenterLeft;
    CBoxEntity *m_pcBoxTopRight, *m_pcBoxBottomRight, *m_pcBoxCenterRight;

    Real m_fObjectiveFunction;
};

#endif
