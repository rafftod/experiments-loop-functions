#ifndef EPUCK_PLANNING_QTOPENGL_USER_FUNCTIONS_H
#define EPUCK_PLANNING_QTOPENGL_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>


#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>


using namespace argos;

class CEPuckPlanningQTOpenGLUserFunctions : public CQTOpenGLUserFunctions {

public:

  CEPuckPlanningQTOpenGLUserFunctions();
  virtual ~CEPuckPlanningQTOpenGLUserFunctions();

  void Draw(CEPuckEntity& c_entity);

private:
  /* Font used for drawing robot's ID */
  QFont m_pcDrawIDFont;

};

#endif
