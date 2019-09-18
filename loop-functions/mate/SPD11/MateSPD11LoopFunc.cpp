/**
  * @file <loop-functions/mate/SPD11/MateSPD11LoopFunc.cpp>
  *
  * @author Fernando Mendiburu - <fmendiburu@ulb.ac.be>
  *
  * @package AutoMoDe-Mate
  *
  * @license MIT License
  */

#include "MateSPD11LoopFunc.h"

/****************************************/
/****************************************/

MateSPD11LoopFunction::MateSPD11LoopFunction() {

  m_fRadius = 0.3;
  m_cCoordBlackSpot1 = CVector2(1.2,0);

  m_fCommunicationDistance = 0.30;
  m_cArenaCenter = CVector2(0,0);
  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

MateSPD11LoopFunction::MateSPD11LoopFunction(const MateSPD11LoopFunction& orig) {}

/****************************************/
/****************************************/

MateSPD11LoopFunction::~MateSPD11LoopFunction() {}

/****************************************/
/****************************************/

void MateSPD11LoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateSPD11LoopFunction::Reset() {
  /* Reset variables */
  m_fObjectiveFunction = 0;
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor MateSPD11LoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());



    return CColor::GRAY50;
}

/****************************************/
/****************************************/

void MateSPD11LoopFunction::PostExperiment() {


    LOG << "Score: " << GetObjectiveFunction() << std::endl;

}

void MateSPD11LoopFunction::PostStep() {

    m_fObjectiveFunction += ComputeObjectiveFunction();


}


/****************************************/
/****************************************/

bool MateSPD11LoopFunction::IsOnSpot1(CVector2& c_position) {

    CVector2 vCurrentPoint(c_position.GetX(), c_position.GetY());

    Real d1 = (m_cCoordBlackSpot1 - vCurrentPoint).Length();

    if (d1 <= m_fRadius) {
      return true;
    }

    return false;
}

/****************************************/
/****************************************/

Real MateSPD11LoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

void MateSPD11LoopFunction::AddNeighs(std::vector<CAgent> &agents, std::vector<CAgent>::iterator ag) {
    for (std::vector<CAgent>::iterator neigh = agents.begin(); neigh != agents.end(); ++neigh) {
        if (neigh->unClusterID != 0)
         continue;
        if ( Distance(ag->cPosition, neigh->cPosition) < m_fCommunicationDistance ) {
            neigh->unClusterID = ag->unClusterID;
            AddNeighs(agents, neigh);
        }
    }
}

/****************************************/
/****************************************/

std::vector<MateSPD11LoopFunction::CAgent> MateSPD11LoopFunction::PickAgents() {

    std::vector<CAgent> agents;
    CSpace::TMapPerType m_tEPuckEntityMap = GetSpace().GetEntitiesByType("epuck");

    CVector2 cEpuckPosition;

    /* create a vector with the agents positions (using the objects CAgents) */
    for (CSpace::TMapPerType::iterator itEPuckEntity = m_tEPuckEntityMap.begin(); itEPuckEntity != m_tEPuckEntityMap.end(); itEPuckEntity++) {
       CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*> (itEPuckEntity->second);
       cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

       /* If you run the experiment in simulation comment the second one, else comment the first one! */
       agents.push_back(CAgent(cEpuckPosition,atoi(pcEpuck->GetId().substr(5, 6).c_str())));
       //agents.push_back(CAgent(cEpuckPosition,atoi(pcEpuck->GetId().substr(pcEpuck->GetId().size()-2, pcEpuck->GetId().size()).c_str())));
    }

    return agents;
}

/****************************************/
/****************************************/

UInt32 MateSPD11LoopFunction::DetermineBiggestGroup(std::vector<CAgent> &agents) {

    /* Cluster the agents in groups */
    UInt32 maxUsedID = 0;
    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
       if (ag->unClusterID != 0)
          continue;
       ag->unClusterID = ++maxUsedID;
       AddNeighs(agents, ag);
    }

    /* Determine the biggest group */
    size_t maxGroupSize = 0;
    UInt32 biggestGroupID = 0;
    for (UInt32 i = 1; i <= maxUsedID; i++){
       size_t size = 0;
       for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){

          if (ag->unClusterID == i) {
             size++;
          }
       }

       if (maxGroupSize < size){
          maxGroupSize = size;
          biggestGroupID = i;
       }
    }

    return biggestGroupID;
}

/****************************************/
/****************************************/

Real MateSPD11LoopFunction::ComputeObjectiveFunction() {

  Real performance = 0;

  /* Push all agents in a vector */
  std::vector<CAgent> agents = PickAgents();

  /* Determine ID group arround the spot */
  UInt32 unGroupID = DetermineBiggestGroup(agents);

  /* Determine the agents forming the biggest region */
  std::vector<CAgent> AgentsBiggestRegion = PickAgentsOfSameID(agents,unGroupID);


  if(!AgentsBiggestRegion.empty()) {

              std::vector<MateSPD11LoopFunction::Edges> vEdges;
              std::vector<UInt32> vNodes;

              /* Determine the edges and nodes of the graph */
              vEdges = DetermineEdges(AgentsBiggestRegion);
              vNodes = DetermineNodes(AgentsBiggestRegion);

              UInt32 un_LSP = 0;
              UInt32 un_SP = 0;


              if(vNodes.size() > 1) {

                /* for each robot calculate the shortest path*/
                for (std::vector<CAgent>::iterator ag = AgentsBiggestRegion.begin(); ag != AgentsBiggestRegion.end(); ++ag){

                    UInt32 unIDSource = ag->unRobotID;

                    un_SP = dijkstraShortestPath(vNodes, vEdges, unIDSource);

                    /* if actual shortest path is higher than the last longest shortest path found before*/
                    if(un_SP > un_LSP ) {
                        un_LSP = un_SP;
                    }
                 }
              }

              performance = un_LSP;
  }

  /* return the longest shortest path: the graph diameter */
  return performance;

}

/****************************************/
/****************************************/

std::vector<UInt32> MateSPD11LoopFunction::DetermineNodes(std::vector<CAgent> agentes) {

    std::vector<UInt32> vNodes;

    for (std::vector<CAgent>::iterator ag = agentes.begin(); ag != agentes.end(); ++ag){
        vNodes.push_back(ag->unRobotID);
    }

    return vNodes;
}

/****************************************/
/****************************************/

std::vector<MateSPD11LoopFunction::Edges> MateSPD11LoopFunction::DetermineEdges(std::vector<CAgent> agents) {

    std::vector<MateSPD11LoopFunction::Edges> vEdges;

    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
        for (std::vector<CAgent>::iterator ag2 = ag; ag2 != agents.end(); ++ag2){

            if(ag->unRobotID != ag2->unRobotID) {
                if(Distance(ag->cPosition, ag2->cPosition) < m_fCommunicationDistance) {
                    //vEdges.push_back(Edges(ag->unRobotID,ag2->unRobotID,Distance(ag->cPosition, ag2->cPosition)));
                    vEdges.push_back(Edges(ag->unRobotID,ag2->unRobotID,1));
                }
            }
        }
    }

    return vEdges;
}

/****************************************/
/****************************************/

std::vector<MateSPD11LoopFunction::CAgent> MateSPD11LoopFunction::PickAgentsOfSameID(std::vector<CAgent> agents, UInt32 un_GroupID) {

    std::vector<CAgent> agentes;

    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
        if(ag->unClusterID == un_GroupID) {
            agentes.push_back(*ag);
        }


    }

    return agentes;
}

/****************************************/
/****************************************/

UInt32 MateSPD11LoopFunction::dijkstraShortestPath(std::vector<UInt32> vNodes, std::vector<MateSPD11LoopFunction::Edges> vEdges, UInt32 unIdSource) {

    const int num_nodes = vNodes.size();

    /* Load edges and weights */
    Edge edge_array[vEdges.size()];
    int weights[vEdges.size()];

    unsigned int i;
    
    for ( i = 0; i < vEdges.size(); i++ ) {
        edge_array[i] = Edge(vEdges[i].InEdge, vEdges[i].OutEdge);
        weights[i] = vEdges[i].fPeso;

    }

    int num_arcs = sizeof(edge_array) / sizeof(Edge);

    // Graph created from the list of edges
    graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);

    // Create property_map from edges to weights
    //boost::property_map<graph_t, boost::edge_weight_t>::type weightmap = get(boost::edge_weight, g);

    // Create vectors to store the predecessors (p) and the distances from the root (d)
    std::vector<vertex_descriptor> p(num_vertices(g));
    std::vector<int> d(num_vertices(g));

    // Create descriptor for the source node
    vertex_descriptor source = vertex(unIdSource, g);

    // Evaluate Dijkstra on graph g with source s, predecessor_map p and distance_map d
    boost::dijkstra_shortest_paths(g, source, boost::predecessor_map(&p[0]).distance_map(&d[0]));

    UInt32 unValue = 0;
    for(i = 0; i < d.size(); i++) {
        if ((unValue < d[i]) && (d[i]<=20)) {  //todo: d[i] is not defined when vertex is not defined, partial sol: d[i]<20
            unValue = d[i];
        }
    }

      /* Print stuff */
//      PrintEdges(vEdges);
//      PrintNodes(vNodes);
//      PrintSource(unIdSource);
//      PrintShortesPath(source,goal,p);

      /* return the maximum shortest path between source and a robot */



      return unValue;

}


/****************************************/
/****************************************/

UInt32 MateSPD11LoopFunction::CountRobots(std::vector<CAgent> agents) {

    UInt32 unNumRobots = 0;

    for (std::vector<CAgent>::iterator ag = agents.begin(); ag != agents.end(); ++ag){
        unNumRobots++;
    }

    return unNumRobots;
}

/****************************************/
/****************************************/

void MateSPD11LoopFunction::PrintShortesPath(vertex_descriptor source, vertex_descriptor goal, std::vector<vertex_descriptor> predecessor) {

    std::vector<boost::graph_traits<graph_t>::vertex_descriptor > path;
    vertex_descriptor current = goal;

    while(current!=source)
    {
        path.push_back(current);
        current = predecessor[current];
    }
    path.push_back(source);

  /* Prints the path */
  std::cout << "Path\n";
  std::vector<boost::graph_traits<graph_t>::vertex_descriptor >::reverse_iterator it;

  for (it = path.rbegin(); it != path.rend(); ++it) {
      std::cout << *it << "->";
  }
  std::cout << "\n";

}

void MateSPD11LoopFunction::PrintEdges(std::vector<MateSPD11LoopFunction::Edges> v_edges) {

    std::vector<MateSPD11LoopFunction::Edges> ::iterator it;

    for (it = v_edges.begin(); it != v_edges.end(); it++) {

        std::cout << "Edge: " << it->InEdge << ", " << it->OutEdge << ": "  << it->fPeso << "\n";
    }


}




void MateSPD11LoopFunction::PrintAgents(std::vector<CAgent> agents) {

    std::vector<CAgent>::iterator it;

    for (it = agents.begin(); it != agents.end(); it++) {

        std::cout << "IdRobot: " << it->unRobotID << " IdCluster: " << it->unClusterID << " pos: " << it->cPosition << "\n";

    }

}


void MateSPD11LoopFunction::PrintNodes(std::vector<UInt32> vNodes) {

    std::vector<UInt32>::iterator it1;

    for (it1 = vNodes.begin(); it1 != vNodes.end(); it1++) {

        std::cout << "node: " << *it1 << "\n";
    }

}


void MateSPD11LoopFunction::PrintSource(UInt32 unIdSource) {

        std::cout << "source: " << unIdSource << "\n";

}


/****************************************/
/****************************************/

CVector3 MateSPD11LoopFunction::GetRandomPosition() {

    CVector3 cPosition;


    do {
    cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                         m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                         0);
    } while(cPosition.Length()>=m_fDistributionRadius);


    return cPosition;
}

REGISTER_LOOP_FUNCTIONS(MateSPD11LoopFunction, "mate_spd11_loop_functions");
