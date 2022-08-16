/*
 * Shelter with Constrained Access
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 */

#ifndef SCA_BIS_LOOP_FUNC_H
#define SCA_BIS_LOOP_FUNC_H

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class SCAbisLoopFunction : public CoreLoopFunctions {

   public:
      SCAbisLoopFunction();
      SCAbisLoopFunction(const SCAbisLoopFunction& orig);
      virtual ~SCAbisLoopFunction();

      virtual void Destroy();
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void PostStep();
      virtual void PostExperiment();


      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

    private:
      bool IsInShelter(CVector2& c_position);

      Real m_fSpotRadius;
      Real m_fWidthShelter;
      Real m_fHeightShelter;
      Real m_fWidthWallsShelter;
      Real m_fHeightWallsShelter;
      CVector2 m_cPositionShelter;
      CVector2 m_cCoordBlackSpot;
      CVector2 m_cCoordWhiteSpot;

      Real m_fObjectiveFunction;
      CBoxEntity *m_pcBoxLeft, *m_pcBoxRight, *m_pcBoxCenter;
};

#endif
