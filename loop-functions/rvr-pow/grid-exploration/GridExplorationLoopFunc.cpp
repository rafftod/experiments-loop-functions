#include "GridExplorationLoopFunc.h"

/****************************************/
/****************************************/

GridExplorationLoopFunction::GridExplorationLoopFunction()
{

    m_arenaSize = 2.50001;
    m_gridSize = 10;
    m_grid.assign(m_gridSize, std::vector<int>(m_gridSize, 0));

    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

GridExplorationLoopFunction::GridExplorationLoopFunction(const GridExplorationLoopFunction &orig) {}

/****************************************/
/****************************************/

GridExplorationLoopFunction::~GridExplorationLoopFunction() {}

/****************************************/
/****************************************/

void GridExplorationLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void GridExplorationLoopFunction::Reset()
{
    CoreLoopFunctions::Reset();

    m_grid.assign(m_gridSize, std::vector<int>(m_gridSize, 0));
    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

void GridExplorationLoopFunction::Init(TConfigurationNode &t_tree)
{
    CoreLoopFunctions::Init(t_tree);
}

argos::CColor GridExplorationLoopFunction::GetFloorColor(const argos::CVector2 &c_position_on_plane)
{
    /*
    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
    if (abs(vCurrentPoint.GetX()) > m_arenaSize/2.0 || abs(vCurrentPoint.GetY()) > m_arenaSize/2.0 ) {
        return CColor::WHITE;
    }
    if ( (m_gridSize*(vCurrentPoint.GetX()/m_arenaSize+0.5)) - floor(m_gridSize*(vCurrentPoint.GetX()/m_arenaSize+0.5)) < 0.1) {
        return CColor::WHITE;
    }
    if ( (m_gridSize*(vCurrentPoint.GetY()/m_arenaSize+0.5)) - floor(m_gridSize*(vCurrentPoint.GetY()/m_arenaSize+0.5)) < 0.1) {
        return CColor::WHITE;
    }
    */
    return CColor::GRAY50;
}

CVector3 GridExplorationLoopFunction::GetRandomPosition()
{
    Real temp;
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    // If b < a, swap them
    if (b < a)
    {
        temp = a;
        a = b;
        b = temp;
    }
    Real fPosX = b * m_fDistributionRadius * cos(2 * CRadians::PI.GetValue() * (a / b));
    Real fPosY = b * m_fDistributionRadius * sin(2 * CRadians::PI.GetValue() * (a / b));

    return CVector3(fPosX, fPosY, 0);
}

/****************************************/
/****************************************/

void GridExplorationLoopFunction::PostStep()
{
    ArrestTrespassers();
    CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0, 0);

    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
    {
        CEPuckEntity *pcEpuck = any_cast<CEPuckEntity *>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        UInt32 X = (UInt32)m_gridSize * (cEpuckPosition.GetX() / m_arenaSize + 0.5);
        UInt32 Y = (UInt32)m_gridSize * (cEpuckPosition.GetY() / m_arenaSize + 0.5);
        if (X < m_gridSize && Y < m_gridSize && X >= 0 && Y >= 0)
        {
            m_grid[X][Y] = 0;
        }
    }

    UInt32 total = 0;
    for (UInt32 i = 0; i < m_gridSize; i++)
    {
        for (UInt32 j = 0; j < m_gridSize; j++)
        {
            total += m_grid[i][j];
            m_grid[i][j] += 1;
        }
    }

    m_fObjectiveFunction += Real(total);
}

/****************************************/
/****************************************/

void GridExplorationLoopFunction::PostExperiment()
{
    m_fObjectiveFunction = m_fObjectiveFunction / m_gridSize / m_gridSize;
    // LOG << "Final value : "<< m_fObjectiveFunction << std::endl;
}

Real GridExplorationLoopFunction::GetObjectiveFunction()
{
    return (m_fObjectiveFunction);
}

void GridExplorationLoopFunction::ArrestTrespassers()
{
    CEPuckEntity *pcEpuck;
    bool bPlaced = false;
    UInt32 unTrials;
    CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("epuck");
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
    {
        pcEpuck = any_cast<CEPuckEntity *>(it->second);
        // Choose position at random
        Real posY = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
        Real posX = pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
        if (pow(posY, 2.0) + pow(posX, 2.0) > pow(1.250, 2.0) && posY < 1.900)
        {
            unTrials = 0;
            do
            {
                ++unTrials;
                CVector3 cEpuckPosition = GetJailPosition();
                bPlaced = MoveEntity(pcEpuck->GetEmbodiedEntity(),
                                     cEpuckPosition,
                                     CQuaternion().FromEulerAngles(m_pcRng->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI)),
                                                                   CRadians::ZERO, CRadians::ZERO),
                                     false);
            } while (!bPlaced && unTrials < 100);
            if (!bPlaced)
            {
                THROW_ARGOSEXCEPTION("Can't place robot");
            }
        }
    }
}

CVector3 GridExplorationLoopFunction::GetJailPosition()
{
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = a * 2.5 - 1.250;
    Real fPosY = b * 0.5 + 1.950;

    return CVector3(fPosX, fPosY, 0);
}

REGISTER_LOOP_FUNCTIONS(GridExplorationLoopFunction, "grid_exploration_loop_functions");
