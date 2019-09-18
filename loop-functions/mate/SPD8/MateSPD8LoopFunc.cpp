/**
  * @file <loop-functions/example/MateSPD8LoopFunc.cpp>
  *
  * @author Antoine Ligot - <aligot@ulb.ac.be>
  *
  * @package ARGoS3-AutoMoDe
  *
  * @license MIT License
  */

#include "MateSPD8LoopFunc.h"

/****************************************/
/****************************************/

MateSPD8LoopFunction::MateSPD8LoopFunction() {

  m_fSideSquare = 1.0;

  m_cCoordSquareSpot = CVector2(0.6, 0);

  m_unNumberPoints = 1000;

  m_fObjectiveFunction = 0;

}

/****************************************/
/****************************************/

MateSPD8LoopFunction::MateSPD8LoopFunction(const MateSPD8LoopFunction& orig) {}

/****************************************/
/****************************************/

MateSPD8LoopFunction::~MateSPD8LoopFunction() {}

/****************************************/
/****************************************/

void MateSPD8LoopFunction::Destroy() {}

/****************************************/
/****************************************/

void MateSPD8LoopFunction::Reset() {
  CoreLoopFunctions::Reset();
}

/****************************************/
/****************************************/

argos::CColor MateSPD8LoopFunction::GetFloorColor(const argos::CVector2& c_position_on_plane) {
  CVector2 cCurrentPoint(c_position_on_plane.GetX(), c_position_on_plane.GetY());

  if (IsOnSquareArea(cCurrentPoint)){
      return CColor::BLACK;
  } else{
      return CColor::GRAY50;
  }
}

/****************************************/
/****************************************/

void MateSPD8LoopFunction::PostExperiment() {
  m_fObjectiveFunction = ComputeObjectiveFunction();
  LOG << "Score: " << GetObjectiveFunction() << std::endl;
}


/****************************************/
/****************************************/

Real MateSPD8LoopFunction::GetObjectiveFunction() {
  return m_fObjectiveFunction;
}

/****************************************/
/****************************************/

Real MateSPD8LoopFunction::ComputeObjectiveFunction() {
    CVector2 cRandomPoint;
    Real dA=0, dP=0;
    CSpace::TMapPerType mEpucks = GetSpace().GetEntitiesByType("epuck");
    CVector2 cEpuckPosition(0,0);
    Real fDistanceToRandomPoint = 0;

    // White square area
    for(UInt32 i = 0; i < m_unNumberPoints; i++){

        Real fMinDistanceOnSquare = 0.67;  // Correspond to worst case, only one robot in the corner of the square

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


    Real performance = 100*100*dA*dA;   // in cm2

    return performance;
}

/****************************************/
/****************************************/

CVector2 MateSPD8LoopFunction::RandomPointOnSquareArea(){
    return CVector2(m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetX() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetX() + m_fSideSquare/2.0f)),
                    m_pcRng->Uniform(CRange<Real>(m_cCoordSquareSpot.GetY() - m_fSideSquare/2.0f, m_cCoordSquareSpot.GetY() + m_fSideSquare/2.0f)));
}

/****************************************/
/****************************************/

bool MateSPD8LoopFunction::IsOnSquareArea(CVector2 c_point){
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

CVector3 MateSPD8LoopFunction::GetRandomPosition() {

    CVector3 cPosition;


    do {
    cPosition = CVector3(m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,-0.6)),
                         m_pcRng->Uniform(CRange<Real>(-m_fDistributionRadius,m_fDistributionRadius)),
                         0);
    } while(cPosition.Length()>=m_fDistributionRadius);


    return cPosition;
}


REGISTER_LOOP_FUNCTIONS(MateSPD8LoopFunction, "mate_spd8_loop_functions");
