/*
 * Shelter with Constrained Access
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 */

#ifndef SCA_BLACK_LOOP_FUNC_H
#define SCA_BLACK_LOOP_FUNC_H

#include "../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class SCAblackLoopFunction : public CoreLoopFunctions {

   public:
      SCAblackLoopFunction();
      SCAblackLoopFunction(const SCAblackLoopFunction& orig);
      virtual ~SCAblackLoopFunction();

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
      CVector2 m_cCoordBlackSpot1;
      CVector2 m_cCoordBlackSpot2;

      Real m_fObjectiveFunction;
      CBoxEntity *m_pcBoxLeft, *m_pcBoxRight, *m_pcBoxCenter;
};

#endif
