#include "ColorAggregationLoopFunc.h"
#include <random>

/****************************************/
/****************************************/

ColorAggregationLoopFunction::ColorAggregationLoopFunction()
{

    m_arenaSize = 2.50001;
    m_gridSize = 10;
    m_patchesSize = 0.3f;
    m_grid.assign(m_gridSize, std::vector<int>(m_gridSize, 0));
    m_unScoreSpot1 = 0;
    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

ColorAggregationLoopFunction::ColorAggregationLoopFunction(const ColorAggregationLoopFunction &orig) {}

/****************************************/
/****************************************/

ColorAggregationLoopFunction::~ColorAggregationLoopFunction() {}

/****************************************/
/****************************************/

void ColorAggregationLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void ColorAggregationLoopFunction::Reset()
{
    CoreLoopFunctions::Reset();
    m_colorMap.clear();
    CreateColorTiles();
    m_unScoreSpot1 = 0;
    m_grid.assign(m_gridSize, std::vector<int>(m_gridSize, 0));
    m_fObjectiveFunction = 0;
}

/****************************************/
/****************************************/

void ColorAggregationLoopFunction::Init(TConfigurationNode &t_tree)
{
    CoreLoopFunctions::Init(t_tree);
    CreateColorTiles();
}

argos::CColor ColorAggregationLoopFunction::GetFloorColor(const argos::CVector2 &c_position_on_plane)
{
    CVector2 vCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());
    for (size_t i = 0; i < m_colorMap.size(); i++)
    {
        for (size_t j = 0; j < m_colorMap[i].size(); j++)
        {
            // check that the square centered at the pair
            // contains the current point
            if (vCurrentPoint.GetX() >= m_colorMap[i][j].first - m_patchesSize / 2 &&
                vCurrentPoint.GetX() <= m_colorMap[i][j].first + m_patchesSize / 2 &&
                vCurrentPoint.GetY() >= m_colorMap[i][j].second - m_patchesSize / 2 &&
                vCurrentPoint.GetY() <= m_colorMap[i][j].second + m_patchesSize / 2)
            {
                return GetColorParameter(i);
            }
        }
    }
    // if (abs(vCurrentPoint.GetX()) > m_arenaSize / 2.0 || abs(vCurrentPoint.GetY()) > m_arenaSize / 2.0)
    // {
    //     return CColor::WHITE;
    // }
    // if ((m_gridSize * (vCurrentPoint.GetX() / m_arenaSize + 0.5)) - floor(m_gridSize * (vCurrentPoint.GetX() / m_arenaSize + 0.5)) < 0.1)
    // {
    //     return CColor::WHITE;
    // }
    // if ((m_gridSize * (vCurrentPoint.GetY() / m_arenaSize + 0.5)) - floor(m_gridSize * (vCurrentPoint.GetY() / m_arenaSize + 0.5)) < 0.1)
    // {
    //     return CColor::WHITE;
    // }

    return CColor::GRAY50;
}

CVector3 ColorAggregationLoopFunction::GetRandomPosition()
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

void ColorAggregationLoopFunction::PostStep()
{
    ArrestTrespassers();
    CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
    CVector2 cEpuckPosition(0, 0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
    {
        CRVREntity *pcEpuck = any_cast<CRVREntity *>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        // check if in a square
        for (size_t i = 0; i < m_colorMap.size(); i++)
        {
            for (size_t j = 0; j < m_colorMap[i].size(); j++)
            {
                // check that the square centered at the pair
                // contains the current point
                if (cEpuckPosition.GetX() >= m_colorMap[i][j].first - m_patchesSize / 2 &&
                    cEpuckPosition.GetX() <= m_colorMap[i][j].first + m_patchesSize / 2 &&
                    cEpuckPosition.GetY() >= m_colorMap[i][j].second - m_patchesSize / 2 &&
                    cEpuckPosition.GetY() <= m_colorMap[i][j].second + m_patchesSize / 2)
                {
                    // m_unScoreSpot1 += GetColorScore(i);
                }
            }
        }
    }
    LOG << "Score: " << m_unScoreSpot1 << std::endl;
}

/****************************************/
/****************************************/

void ColorAggregationLoopFunction::PostExperiment()
{
    ArrestTrespassers();
    CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
    CVector2 cEpuckPosition(0, 0);
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
    {
        CRVREntity *pcEpuck = any_cast<CRVREntity *>(it->second);
        cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                           pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        // check if in a square
        for (size_t i = 0; i < m_colorMap.size(); i++)
        {
            for (size_t j = 0; j < m_colorMap[i].size(); j++)
            {
                // check that the square centered at the pair
                // contains the current point
                if (cEpuckPosition.GetX() >= m_colorMap[i][j].first - m_patchesSize / 2 &&
                    cEpuckPosition.GetX() <= m_colorMap[i][j].first + m_patchesSize / 2 &&
                    cEpuckPosition.GetY() >= m_colorMap[i][j].second - m_patchesSize / 2 &&
                    cEpuckPosition.GetY() <= m_colorMap[i][j].second + m_patchesSize / 2)
                {
                    m_unScoreSpot1 += GetColorScore(i);
                }
            }
        }
    }
    m_fObjectiveFunction = m_unScoreSpot1;
    LOG << "Score: " << m_unScoreSpot1 << std::endl;
    LOG << "Final value : " << m_fObjectiveFunction << std::endl;
}

Real ColorAggregationLoopFunction::GetObjectiveFunction()
{
    return (m_fObjectiveFunction);
}

void ColorAggregationLoopFunction::ArrestTrespassers()
{
    CRVREntity *pcEpuck;
    bool bPlaced = false;
    UInt32 unTrials;
    CSpace::TMapPerType &tEpuckMap = GetSpace().GetEntitiesByType("rvr");
    for (CSpace::TMapPerType::iterator it = tEpuckMap.begin(); it != tEpuckMap.end(); ++it)
    {
        pcEpuck = any_cast<CRVREntity *>(it->second);
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

CVector3 ColorAggregationLoopFunction::GetJailPosition()
{
    Real a = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));
    Real b = m_pcRng->Uniform(CRange<Real>(0.0f, 1.0f));

    Real fPosX = a * 2.5 - 1.250;
    Real fPosY = b * 0.5 + 1.950;

    return CVector3(fPosX, fPosY, 0);
}

void ColorAggregationLoopFunction::CreateColorTiles()
{
    std::vector<std::pair<Real, Real>> TilePositions = {
        // upper row
        std::pair<Real, Real>(0.75, -0.25), std::pair<Real, Real>(0.75, 0.25),
        // upper mid row
        std::pair<Real, Real>(0.25, -0.75), std::pair<Real, Real>(0.25, -0.25), std::pair<Real, Real>(0.25, 0.25), std::pair<Real, Real>(0.25, 0.75),
        // lower mid row
        std::pair<Real, Real>(-0.25, -0.75), std::pair<Real, Real>(-0.25, -0.25), std::pair<Real, Real>(-0.25, 0.25), std::pair<Real, Real>(-0.25, 0.75),
        // lower row
        std::pair<Real, Real>(-0.75, -0.25), std::pair<Real, Real>(-0.75, 0.25)};
    // assign each tile a position at random from the list
    for (size_t i = 0; i < 3; i++)
    {
        std::vector<std::pair<Real, Real>> ptColorVector;
        for (size_t j = 0; j < 4; j++)
        {
            size_t k = m_pcRng->Uniform(CRange<UInt32>(0, TilePositions.size()));
            ptColorVector.push_back(TilePositions[k]);
            TilePositions.erase(TilePositions.begin() + k);
        }
        m_colorMap.push_back(ptColorVector);
    }
}

CColor ColorAggregationLoopFunction::GetColorParameter(const UInt32 &un_value)
{
    CColor cColorParameter;
    switch (un_value)
    {
    case 0:
        cColorParameter = CColor::GREEN;
        break;
    case 1:
        // red physical patches
        cColorParameter = CColor::RED;
        break;
    case 2:
        // yellow physical patches
        cColorParameter = CColor::YELLOW;
        break;
    default:
        cColorParameter = CColor::BLACK;
    }
    return cColorParameter;
}

Real ColorAggregationLoopFunction::GetColorScore(const UInt32 &un_value)
{
    return un_value;
}

REGISTER_LOOP_FUNCTIONS(ColorAggregationLoopFunction, "color_aggregation_loop_functions");
