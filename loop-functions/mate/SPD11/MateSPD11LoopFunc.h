/*
 * Longest Shortest Path
 *
 * @file <loop-functions/mate/SPD11/MateSPD11LoopFunc.cpp>
 *
 * @author Fernando Mendiburu - <fmendibu@ulb.ac.be>
 *
 * @package AutoMoDe-Mate
 *
 * @license MIT License
 */

#ifndef MATE_SPD11_LOOP_FUNC_H
#define MATE_SPD11_LOOP_FUNC_H

#include "../../../src/CoreLoopFunctions.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <limits>
#include <numeric>

#include <boost/config.hpp>
#include <iostream>
#include <fstream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

typedef boost::adjacency_list <boost::listS, boost::vecS, boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, int> > graph_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;

using namespace argos;

class MateSPD11LoopFunction : public CoreLoopFunctions {

   public:
      MateSPD11LoopFunction();
      MateSPD11LoopFunction(const MateSPD11LoopFunction& orig);
      virtual ~MateSPD11LoopFunction();

      virtual void Destroy();
      virtual void Reset();
      virtual void PostExperiment();
      virtual void PostStep();


      Real GetObjectiveFunction();

      virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

      virtual CVector3 GetRandomPosition();   


      private:

      Real m_fRadius;
      CVector2 m_cCoordBlackSpot1;

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


      struct Edges {
          UInt32 InEdge;
          UInt32 OutEdge;
          //Real fPeso;
          int fPeso;
          Edges(UInt32 s_InEdge, UInt32 s_OutEdge, Real f_peso) {
              InEdge = s_InEdge;
              OutEdge = s_OutEdge;
              fPeso = f_peso;
          }
      };

      Real ComputeObjectiveFunction();

      UInt32 CountRobots(std::vector<CAgent> agents);

      bool IsOnSpot1(CVector2& c_position);

      void AddNeighs(std::vector<CAgent> &agents, std::vector<CAgent>::iterator ag);

      UInt32 dijkstraShortestPath(std::vector<UInt32> vNodes, std::vector<Edges> vVertices, UInt32 unIdSource);

      std::vector<CAgent> PickAgents();

      std::vector<CAgent> PickAgentsOfSameID(std::vector<CAgent> agents, UInt32 un_GroupID);

      UInt32 DetermineBiggestGroup(std::vector<CAgent> &agents);

      std::vector<Edges> DetermineEdges(std::vector<CAgent> agents);

      std::vector<UInt32> DetermineNodes(std::vector<CAgent> agentes);

      void PrintAgents(std::vector<CAgent> agents);

      void PrintEdges(std::vector<Edges> v_edges);

      void PrintNodes(std::vector<UInt32> vNodes);

      void PrintSource(UInt32 unIdSource);

      void PrintShortesPath(vertex_descriptor source, vertex_descriptor goal, std::vector<vertex_descriptor> predecessor);

      Real m_fRadiusRobot;

      Real m_fObjectiveFunction;

      Real m_fCommunicationDistance;

      UInt32 m_unNumberPoints;

      CVector2 m_cArenaCenter;

      UInt32 m_unLengthExperiment;
};

#endif
