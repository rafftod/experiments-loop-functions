/**
  * @file <loop-functions/example/MateSCLoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "MateSCLoopFunc.h"

/****************************************/
/****************************************/

MateSCLoopFunction::MateSCLoopFunction() {

  m_fSideSquare = 0.8;

  m_cCoordSquareSpot = CVector2(-0.6, 0);

  m_unNumberPoints = 1000;

  m_fObjectiveFunction = 0;
  m_fDoptA = 0.08;
}

/****************************************/
/****************************************/

MateSCLoopFunction::MateSCLoopFunction(const MateSCLoopFunction& orig) {}

/****************************************/
/****************************************/

MateSCLoopFunction::~MateSCLoopFunction() {}

/****************************************/
/****************************************/

void MateSCLoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateSCLoopFunction::Reset() {
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor MateSCLoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 cCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  if (IsOnSquareArea(cCurrentPoint)){
      return CColor::BLACK;
  } else{
      return CColor::GRAY50;
  }
}

/****************************************/
/****************************************/

void MateSCLoopFunction::PostExperiment() {
  m_fObjectiveFunction = ComputeObjectiveFunction();
  LOG << "Score: " << GetObjectiveFunction() << std::endl;
}

/****************************************/
/****************************************/

Real MateSCLoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

Real MateSCLoopFunction::ComputeObjectiveFunction() {
    CVector2 cRandomPoint;
    Real dA=0;
    CSpace::TMapPerType mEpucks = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    Real fDistanceToRandomPoint = 0;

    // White square area
    for(UInt32 i = 0; i < m_unNumberPoints; i++){

        //Real fMinDistanceOnSquare = 0.85;  // Correspond to the diagonal of the square area
        Real fMinDistanceOnSquare = 1.13;  // Correspond to the diagonal of the square area

        cRandomPoint = RandomPointOnSquareArea();

        for (CSpace::TMapPerType::iterator it = mEpucks.begin(); it != mEpucks.end(); ++it) {
            CEPuckEntity* pcEpuck = any_cast<CEPuckEntity*> ((*it).second);
            cEpuckPosition.Set(pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                               pcEpuck->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            if(IsOnSquareArea(cEpuckPosition)){
                fDistanceToRandomPoint = (cRandomPoint - cEpuckPosition).Length();
                if(fDistanceToRandomPoint < fMinDistanceOnSquare){
                    fMinDistanceOnSquare = fDistanceToRandomPoint;
                }
            }
        }

        dA += fMinDistanceOnSquare;
    }
    dA /= m_unNumberPoints;


    Real performance = (dA);

    return performance;
}

/****************************************/
/****************************************/

CVector2 MateSCLoopFunction::RandomPointOnSquareArea(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetX() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare/2.0f)),
                    m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetY() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare/2.0f)));
}

/****************************************/
/****************************************/

bool MateSCLoopFunction::IsOnSquareArea(CVector2 c_point){
    CRange<Real> cRangeSquareX(m_cCoordSquareSpot.GetX() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare/2.0f);
    CRange<Real> cRangeSquareY(m_cCoordSquareSpot.GetY() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare/2.0f);

    if (cRangeSquareX.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetX()) &&
            cRangeSquareY.WithinMinBoundIncludedMaxBoundIncluded(c_point.GetY())) {
        return true;
    }
    return false;
}


/****************************************/
/****************************************/

CVector3 MateSCLoopFunction::GetRandomPosition() {

    CVector3 cPosition;

    cPosition.FromSphericalCoords(
                m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius, m_fDistributionRadius)),     // distance from origin
                CRadians::PI_OVER_TWO,                                                             // angle with Z axis
                m_pcRng->Uniform(CRadians::UNSIGNED_RANGE)                                         // rotation around Z
                );

    return CVector3(cPosition.GetX(), cPosition.GetY(), 0);
}


REGISTER_LOOP_FUNCTIONS(MateSCLoopFunction, "mate_sc_loop_functions");
