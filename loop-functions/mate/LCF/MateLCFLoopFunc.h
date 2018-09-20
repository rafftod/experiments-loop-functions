/*
 * Largest Covering Area
 *
 * @file <loop-functions/mate/LCF/MateLCFLoopFunc.cpp>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package AutoMoDe-Mate
 *
 * @license MIT License
 */

#ifndef MATE_LCF_LOOP_FUNC_H
#define MATE_LCF_LOOP_FUNC_H

#include "../../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <limits>
#include <numeric>

using namespace argos;

class MateLCFLoopFunction : public CoreLoopFunctions {

   public:
      MateLCFLoopFunction();
      MateLCFLoopFunction(const MateLCFLoopFunction& orig);
      virtual ~MateLCFLoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostExperiment();
      virtual void PostStep();

      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

      virtual CVector2 GetRandomPoint();

      virtual CVector2 RandomPointOnNest();


      private:

      struct CAgent {
        CVector2 cPosition;
        UInt32 unRobotID;
        UInt32 unClusterID;
        CAgent(CVector2 c_position, UInt32 un_RobotID) {
          cPosition = c_position;
          unRobotID = un_RobotID;
          unClusterID = 0;
        }
        CAgent(CVector2 c_position, UInt32 un_RobotID, UInt32 un_GroupID) {
          cPosition = c_position;
          unRobotID = un_RobotID;
          unClusterID = un_GroupID;
        }

      };

      Real ComputeObjectiveFunction();

      bool IsOnArea(CVector2& c_position);

      bool IsInsideArena(CVector2& c_position);

      void AddNeighs(std::vector<CAgent> &agents, std::vector<CAgent>::iterator ag);

      std::vector<MateLCFLoopFunction::CAgent> PickAgents();

      std::vector<MateLCFLoopFunction::CAgent> PickAgentsOfSameID(std::vector<CAgent> agents, UInt32 un_GroupID);

      UInt32 DetermineBiggestGroup(std::vector<CAgent> &agents);

      Real ComputeCoverageRatio(std::vector<CAgent> &agents, UInt32 GroupID);

      void PrintAgents(std::vector<CAgent> agents);

      Real m_fRadiusRobot;

      Real m_fObjectiveFunction;

      Real m_fSensingRange;

      Real m_fCommunicationDistance;

      UInt32 m_unNumberPoints;

      CVector2 m_cArenaCenter;

      Real m_fAreaLimit;

};

#endif
