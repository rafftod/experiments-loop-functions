/*
 * Swarm Coverage Region
 *
 * @file <loop-functions/example/MateSPD8LoopFunc.cpp>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package AutoMoDe-Mate
 *
 * @license MIT License
 */

 #ifndef MATE_SPD8_LOOP_FUNC_H
 #define MATE_SPD8_LOOP_FUNC_H

 #include "../../../src/CoreLoopFunctions.h"
 #include <argos3/core/simulator/space/space.h>
 #include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class MateSPD8LoopFunction : public CoreLoopFunctions {

   public:
      MateSPD8LoopFunction();
      MateSPD8LoopFunction(const MateSPD8LoopFunction& orig);
      virtual ~MateSPD8LoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostExperiment();

      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();

    private:
      Real ComputeObjectiveFunction();
      CVector2 RandomPointOnSquareArea();
      CVector2 RandomPointOnCirclePerimeter();
      bool IsOnSquareArea(CVector2 c_point);
      bool IsOnCirclePerimeter(CVector2 c_point);

      Real m_fRadiusRobot;

      Real m_fSideSquare;

      CVector2 m_cCoordSquareSpot;

      UInt32 m_unNumberPoints;

      Real m_fObjectiveFunction;

};

#endif
